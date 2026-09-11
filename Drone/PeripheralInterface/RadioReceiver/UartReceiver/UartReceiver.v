`timescale 1ns / 1ps

// Radio serial transport: idle-high UART, 8N1, LSB first.
// All logic stays on clk; rst is synchronous and active high.
module UartReceiver #(
    parameter integer CLK_FREQ = 100_000_000,
    parameter integer BAUD_RATE = 115_200
) (
    input wire clk,
    input wire rst,
    input wire rx,
    output reg [7:0] data,
    output reg data_valid,
    output reg framing_error
);
    localparam integer BIT_CYCLES =
        (BAUD_RATE > 0) ? (CLK_FREQ + BAUD_RATE / 2) / BAUD_RATE : 1;
    localparam integer WIDTH = (BIT_CYCLES > 1) ? $clog2(BIT_CYCLES) : 1;
    localparam [WIDTH-1:0] BIT_LAST = BIT_CYCLES - 1;
    localparam [WIDTH-1:0] HALF_LAST = BIT_CYCLES / 2 - 1;
    localparam [2:0] IDLE = 0, START = 1, DATA = 2, STOP = 3, WAIT_HIGH = 4;

    (* ASYNC_REG = "TRUE" *) reg rx_meta, rx_sync;
    reg rx_previous;
    reg [2:0] state, bit_index;
    reg [WIDTH-1:0] countdown;
    reg [7:0] shift;

    // synthesis translate_off
    initial begin
        if (CLK_FREQ <= 0 || BAUD_RATE <= 0 || BIT_CYCLES < 8)
            $fatal(1, "UartReceiver: require at least 8 clocks per UART bit");
    end
    // synthesis translate_on

    always @(posedge clk) begin
        if (rst) begin
            rx_meta <= 1'b1;
            rx_sync <= 1'b1;
            rx_previous <= 1'b1;
        end else begin
            rx_meta <= rx;
            rx_sync <= rx_meta;
            rx_previous <= rx_sync;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            countdown <= 0;
            bit_index <= 0;
            shift <= 0;
            data <= 0;
            data_valid <= 0;
            framing_error <= 0;
        end else begin
            data_valid <= 0;
            framing_error <= 0;
            case (state)
                IDLE: if (rx_previous && !rx_sync) begin
                    countdown <= HALF_LAST;
                    state <= START;
                end
                START: if (countdown != 0)
                    countdown <= countdown - 1'b1;
                else if (!rx_sync) begin
                    countdown <= BIT_LAST;
                    bit_index <= 0;
                    state <= DATA;
                end else state <= IDLE; // Reject a short low glitch.
                DATA: if (countdown != 0)
                    countdown <= countdown - 1'b1;
                else begin
                    shift[bit_index] <= rx_sync;
                    countdown <= BIT_LAST;
                    if (bit_index == 7) state <= STOP;
                    else bit_index <= bit_index + 1'b1;
                end
                STOP: if (countdown != 0)
                    countdown <= countdown - 1'b1;
                else if (rx_sync) begin
                    data <= shift;
                    data_valid <= 1;
                    state <= IDLE;
                end else begin
                    framing_error <= 1;
                    state <= WAIT_HIGH;
                end
                WAIT_HIGH: if (rx_sync) state <= IDLE;
                default: state <= IDLE;
            endcase
        end
    end
endmodule
