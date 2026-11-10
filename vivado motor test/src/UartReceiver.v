`timescale 1ns / 1ps

// Shared peripheral serial transport: idle-high UART, 8N1, LSB first.
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
    // Count system-clock edges to time UART bits; no separate baud clock is used.
    // Round to the nearest whole cycle: 100 MHz / 115200 baud = about 868
    // clocks per bit (8.68 us). Half a bit is 434 clocks with these defaults.
    localparam integer BIT_CYCLES =
        (BAUD_RATE > 0) ? (CLK_FREQ + BAUD_RATE / 2) / BAUD_RATE : 1;
    localparam integer WIDTH = (BIT_CYCLES > 1) ? $clog2(BIT_CYCLES) : 1;
    // Loading N - 1 gives N clocks of delay because zero is also a count.
    localparam [WIDTH-1:0] BIT_LAST = BIT_CYCLES - 1;
    localparam [WIDTH-1:0] HALF_LAST = BIT_CYCLES / 2 - 1;
    localparam [2:0] IDLE = 0, START = 1, DATA = 2, STOP = 3, WAIT_HIGH = 4;

    // RX arrives asynchronously. Two flip-flops reduce the chance of
    // metastability reaching the state machine; only rx_sync is decoded.
    (* ASYNC_REG = "TRUE" *) reg rx_meta, rx_sync;
    // The extra history register lets us detect a falling edge on rx_sync.
    reg rx_previous;
    reg [2:0] state, bit_index;
    reg [WIDTH-1:0] countdown;
    reg [7:0] shift; // Build the byte privately until its stop bit is checked.

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
            // Default low makes these one-clock pulses. data holds the last
            // accepted byte, so downstream logic reads it only on data_valid.
            data_valid <= 0;
            framing_error <= 0;
            case (state)
                // UART idles high. A falling edge starts timing a new byte.
                IDLE: if (rx_previous && !rx_sync) begin
                    countdown <= HALF_LAST;
                    state <= START;
                end
                // Check halfway through the start bit, away from its edges.
                // If still low, wait one full bit to reach the center of D0.
                START: if (countdown != 0)
                    countdown <= countdown - 1'b1;
                else if (!rx_sync) begin
                    countdown <= BIT_LAST;
                    bit_index <= 0;
                    state <= DATA;
                end else state <= IDLE; // Reject a short low glitch.
                // Capture one bit at each bit center, D0 first through D7.
                // The synchronizer runs every clock; byte assembly happens
                // only when this countdown expires (one sample per data bit).
                DATA: if (countdown != 0)
                    countdown <= countdown - 1'b1;
                else begin
                    shift[bit_index] <= rx_sync;
                    countdown <= BIT_LAST;
                    if (bit_index == 7) state <= STOP;
                    else bit_index <= bit_index + 1'b1;
                end
                // One bit after D7 is the stop-bit center. 8N1 means eight
                // data bits, no parity, and one high stop bit.
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
                // After a bad stop bit, wait for idle before looking for a
                // fresh falling edge; a held-low line must not produce bytes.
                WAIT_HIGH: if (rx_sync) state <= IDLE;
                default: state <= IDLE;
            endcase
        end
    end
endmodule
