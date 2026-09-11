`timescale 1ns / 1ps

// Radio packet decoder: FS-iA6B servo iBUS, 14 little-endian channels.
// channels[15:0] is CH1, channels[31:16] is CH2, ... CH14 at [223:208].
module IBusDecoder #(
    parameter integer CLK_FREQ = 100_000_000,
    parameter integer TIMEOUT_MS = 100,
    parameter integer BYTE_TIMEOUT_US = 500
) (
    input wire clk,
    input wire rst,
    input wire [7:0] data,
    input wire data_valid,
    input wire framing_error,
    output reg [223:0] channels,
    output reg frame_valid,
    output reg rc_valid
);
    // Wide intermediate arithmetic avoids overflow at 100 MHz.
    localparam integer TIMEOUT_CYCLES = (64'd1 * CLK_FREQ * TIMEOUT_MS) / 1000;
    localparam integer GAP_CYCLES = (64'd1 * CLK_FREQ * BYTE_TIMEOUT_US) / 1_000_000;
    localparam integer TIMEOUT_WIDTH = (TIMEOUT_CYCLES > 1) ? $clog2(TIMEOUT_CYCLES) : 1;
    localparam integer GAP_WIDTH = (GAP_CYCLES > 1) ? $clog2(GAP_CYCLES) : 1;
    localparam [TIMEOUT_WIDTH-1:0] TIMEOUT_LAST = TIMEOUT_CYCLES - 1;
    localparam [GAP_WIDTH-1:0] GAP_LAST = GAP_CYCLES - 1;

    reg [TIMEOUT_WIDTH-1:0] age;
    reg [GAP_WIDTH-1:0] gap;
    reg [5:0] position;
    reg [223:0] pending_channels;
    reg [15:0] checksum;
    reg [7:0] checksum_low;
    wire accept_frame = data_valid && !framing_error && gap != GAP_LAST &&
                        position == 31 && {data, checksum_low} == checksum;

    // synthesis translate_off
    initial begin
        if (CLK_FREQ <= 0 || TIMEOUT_MS <= 0 || BYTE_TIMEOUT_US <= 0 ||
            TIMEOUT_CYCLES < 1 || GAP_CYCLES < 2)
            $fatal(1, "IBusDecoder: invalid clock or timeout parameters");
    end
    // synthesis translate_on

    always @(posedge clk) begin
        if (rst) begin
            channels <= 0;
            pending_channels <= 0;
            frame_valid <= 0;
            rc_valid <= 0;
            age <= 0;
            gap <= GAP_LAST;
            position <= 0;
            checksum <= 0;
            checksum_low <= 0;
        end else begin
            frame_valid <= accept_frame;
            if (accept_frame) begin
                channels <= pending_channels; // Atomic commit of all 14 channels.
                rc_valid <= 1;
                age <= 0;
            end else if (rc_valid) begin
                if (age == TIMEOUT_LAST) rc_valid <= 0;
                else age <= age + 1'b1;
            end

            if (framing_error) begin
                position <= 0;
                gap <= GAP_LAST;
            end else if (data_valid) begin
                gap <= 0;
                // A long byte gap abandons any incomplete packet. The arriving
                // byte can immediately start a new frame.
                if (gap == GAP_LAST || position == 0) begin
                    if (data == 8'h20) begin
                        position <= 1;
                        checksum <= 16'hffdf; // 0xffff - 0x20
                    end else position <= 0;
                end else if (position == 1) begin
                    if (data == 8'h40) begin
                        checksum <= checksum - 16'h0040;
                        position <= 2;
                    end else if (data == 8'h20) begin
                        checksum <= 16'hffdf; // Overlapping header candidate.
                        position <= 1;
                    end else position <= 0;
                end else if (position < 30) begin
                    pending_channels[(position - 2) * 8 +: 8] <= data;
                    checksum <= checksum - {8'b0, data};
                    position <= position + 1'b1;
                end else if (position == 30) begin
                    checksum_low <= data;
                    position <= 31;
                end else begin
                    position <= 0; // Good or bad checksum: seek the next header.
                end
            end else if (gap != GAP_LAST) begin
                gap <= gap + 1'b1;
            end else position <= 0;
        end
    end
endmodule
