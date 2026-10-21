`timescale 1ns / 1ps

// Shared by System and PeripheralInterface: idle-high UART, 8N1, LSB first.
// Pulse start only while busy is low.
module UartTransmitter #(
    parameter integer CLK_FREQ = 100_000_000,
    parameter integer BAUD_RATE = 115_200
) (
    input wire clk,
    input wire rst,
    input wire [7:0] data,
    input wire start,
    output wire tx,
    output reg busy
);
    localparam integer BIT_CYCLES =
        (BAUD_RATE > 0) ? (CLK_FREQ + BAUD_RATE / 2) / BAUD_RATE : 1;
    localparam integer WIDTH = (BIT_CYCLES > 1) ? $clog2(BIT_CYCLES) : 1;
    reg [WIDTH-1:0] countdown;
    reg [9:0] shift;
    reg [3:0] bit_index;
    assign tx = busy ? shift[0] : 1'b1;

    // synthesis translate_off
    initial begin
        if (CLK_FREQ <= 0 || BAUD_RATE <= 0 || BIT_CYCLES < 8)
            $fatal(1, "UartTransmitter: require at least 8 clocks per UART bit");
    end
    // synthesis translate_on

    always @(posedge clk) begin
        if (rst) begin
            busy <= 0;
            shift <= 10'h3ff;
            bit_index <= 0;
            countdown <= 0;
        end else if (!busy) begin
            if (start) begin
                shift <= {1'b1, data, 1'b0};
                bit_index <= 0;
                countdown <= BIT_CYCLES - 1;
                busy <= 1;
            end
        end else if (countdown != 0)
            countdown <= countdown - 1'b1;
        else if (bit_index == 9)
            busy <= 0;
        else begin
            shift <= {1'b1, shift[9:1]};
            bit_index <= bit_index + 1'b1;
            countdown <= BIT_CYCLES - 1;
        end
    end
endmodule
