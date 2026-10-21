`timescale 1ns / 1ps

// Byte stream -> queued UART output. A byte is accepted on valid && ready.
// Keep data and valid stable while stalled. rst discards queued/in-flight bytes.
module UartOutput #(
    parameter integer CLK_FREQ = 100_000_000,
    parameter integer BAUD_RATE = 115_200,
    parameter integer FIFO_DEPTH = 16
) (
    input wire clk,
    input wire rst,
    input wire [7:0] data,
    input wire valid,
    output wire ready,
    output wire busy,
    output wire tx
);
    localparam integer POINTER_WIDTH = (FIFO_DEPTH > 1) ? $clog2(FIFO_DEPTH) : 1;
    localparam integer COUNT_WIDTH = (FIFO_DEPTH > 1) ? $clog2(FIFO_DEPTH + 1) : 1;
    reg [7:0] fifo [0:FIFO_DEPTH-1];
    reg [POINTER_WIDTH-1:0] read_pointer, write_pointer;
    reg [COUNT_WIDTH-1:0] count;
    wire transmitting;
    wire pop = !rst && !transmitting && count != 0;
    assign ready = !rst && (count < FIFO_DEPTH || pop);
    wire push = valid && ready;
    assign busy = transmitting || count != 0;

    UartTransmitter #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)) transmitter (
        .clk(clk), .rst(rst), .data(fifo[read_pointer]), .start(pop),
        .busy(transmitting), .tx(tx)
    );

    // synthesis translate_off
    initial begin
        if (FIFO_DEPTH < 1) $fatal(1, "UartOutput: FIFO_DEPTH must be positive");
    end
    // synthesis translate_on

    always @(posedge clk) begin
        if (rst) begin
            read_pointer <= 0;
            write_pointer <= 0;
            count <= 0;
        end else begin
            if (push) begin
                fifo[write_pointer] <= data;
                write_pointer <= (write_pointer == FIFO_DEPTH - 1) ?
                    0 : write_pointer + 1'b1;
            end
            if (pop) begin
                read_pointer <= (read_pointer == FIFO_DEPTH - 1) ?
                    0 : read_pointer + 1'b1;
            end
            case ({push, pop})
                2'b10: count <= count + 1'b1;
                2'b01: count <= count - 1'b1;
                default: count <= count;
            endcase
        end
    end
endmodule
