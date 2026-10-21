`timescale 1ns / 1ps

// Basys3 common-anode display. Physical seg, dp and an signals are active low.
// write latches all four hex digits, enables and decimal points together.
module SevenSegmentDisplay #(
    parameter integer CLK_FREQ = 100_000_000,
    parameter integer REFRESH_HZ = 250, // Complete four-digit sweeps per second.
    parameter integer BLANK_CYCLES = (CLK_FREQ / 100_000 < 2) ? 2 : CLK_FREQ / 100_000
) (
    input wire clk,
    input wire rst,
    input wire write,
    input wire [15:0] value, // Low nibble is the rightmost digit (AN0).
    input wire [3:0] digit_enable, // Logical 1 means visible.
    input wire [3:0] decimal_points, // Logical 1 means lit.
    output reg [6:0] seg, // seg[0]=A through seg[6]=G.
    output reg dp,
    output reg [3:0] an
);
    localparam integer SLOT_CYCLES = (REFRESH_HZ > 0) ? CLK_FREQ / (4 * REFRESH_HZ) : 1;
    localparam integer WIDTH = (SLOT_CYCLES > 1) ? $clog2(SLOT_CYCLES) : 1;
    reg [WIDTH-1:0] counter;
    reg [1:0] digit;
    reg [15:0] pending_value, active_value;
    reg [3:0] pending_enable, active_enable, pending_points, active_points;

    function [6:0] hex_segments;
        input [3:0] hex;
        begin
            // Bit order G F E D C B A; zero lights a segment.
            case (hex)
                4'h0: hex_segments = 7'b1000000;
                4'h1: hex_segments = 7'b1111001;
                4'h2: hex_segments = 7'b0100100;
                4'h3: hex_segments = 7'b0110000;
                4'h4: hex_segments = 7'b0011001;
                4'h5: hex_segments = 7'b0010010;
                4'h6: hex_segments = 7'b0000010;
                4'h7: hex_segments = 7'b1111000;
                4'h8: hex_segments = 7'b0000000;
                4'h9: hex_segments = 7'b0010000;
                4'ha: hex_segments = 7'b0001000;
                4'hb: hex_segments = 7'b0000011;
                4'hc: hex_segments = 7'b1000110;
                4'hd: hex_segments = 7'b0100001;
                4'he: hex_segments = 7'b0000110;
                4'hf: hex_segments = 7'b0001110;
                default: hex_segments = 7'b1111111;
            endcase
        end
    endfunction

    // synthesis translate_off
    initial begin
        if (CLK_FREQ <= 0 || REFRESH_HZ <= 0 || BLANK_CYCLES < 2 ||
            SLOT_CYCLES <= BLANK_CYCLES)
            $fatal(1, "SevenSegmentDisplay: require a slot longer than BLANK_CYCLES >= 2");
    end
    // synthesis translate_on

    always @(posedge clk) begin
        if (rst) begin
            pending_value <= 0;
            pending_enable <= 0;
            pending_points <= 0;
            active_value <= 0;
            active_enable <= 0;
            active_points <= 0;
            counter <= SLOT_CYCLES - 1;
            digit <= 3;
            seg <= 7'h7f;
            dp <= 1;
            an <= 4'hf;
        end else begin
            if (write) begin
                pending_value <= value;
                pending_enable <= digit_enable;
                pending_points <= decimal_points;
            end
            if (counter == SLOT_CYCLES - 1) begin
                counter <= 0;
                an <= 4'hf; // Turn off the previous digit before changing segments.
                digit <= digit + 1'b1;
                if (digit == 3) begin
                    // Snapshot once per sweep so writes cannot mix old/new digits.
                    // A coincident write takes effect immediately in this sweep.
                    active_value <= write ? value : pending_value;
                    active_enable <= write ? digit_enable : pending_enable;
                    active_points <= write ? decimal_points : pending_points;
                end
            end else begin
                counter <= counter + 1'b1;
                if (counter == BLANK_CYCLES - 2) begin
                    // Leave one further clock for segment outputs to settle.
                    seg <= hex_segments(active_value[digit*4 +: 4]);
                    dp <= !active_points[digit];
                end
                if (counter == BLANK_CYCLES - 1)
                    an <= active_enable[digit] ? ~(4'b0001 << digit) : 4'hf;
            end
        end
    end
endmodule
