`timescale 1ns / 1ps

// One unidirectional DShot ESC channel. Inputs are synchronous to clk.
// send is accepted only while !busy. The entire packet is latched at acceptance.
module MotorDriver #(
    parameter integer CLK_FREQ = 100_000_000,
    parameter integer DSHOT_RATE = 300_000
) (
    input wire clk,
    input wire rst,
    input wire send,
    input wire enable,
    input wire [10:0] throttle,
    output reg esc_signal,
    output reg busy
);
    localparam integer BIT_CYCLES = (DSHOT_RATE > 0) ?
        (CLK_FREQ + DSHOT_RATE / 2) / DSHOT_RATE : 1;
    localparam integer ZERO_HIGH = (BIT_CYCLES * 3 + 4) / 8;
    localparam integer ONE_HIGH = (BIT_CYCLES * 6 + 4) / 8;
    // Include two full low bit periods before accepting another packet.
    localparam integer GAP_CYCLES = 2 * BIT_CYCLES;
    localparam integer COUNTER_WIDTH = (GAP_CYCLES > 1) ? $clog2(GAP_CYCLES) : 1;
    reg [COUNTER_WIDTH-1:0] counter;
    reg [15:0] packet;
    reg [3:0] bit_index;
    reg gap;
    // 1..47 are ESC configuration commands, never throttle. Reject them.
    wire [10:0] safe_throttle = (enable && throttle >= 11'd48) ? throttle : 11'd0;
    wire [11:0] payload = {safe_throttle, 1'b0}; // No telemetry request.
    wire [3:0] checksum = payload[11:8] ^ payload[7:4] ^ payload[3:0];
    wire [15:0] next_packet = {payload, checksum};

    // synthesis translate_off
    initial begin
        if (CLK_FREQ <= 0 || (DSHOT_RATE != 150_000 &&
            DSHOT_RATE != 300_000 && DSHOT_RATE != 600_000))
            $fatal(1, "MotorDriver: require positive CLK_FREQ and DShot150/300/600");
        if (BIT_CYCLES < 8 || ZERO_HIGH < 1 || ONE_HIGH >= BIT_CYCLES)
            $fatal(1, "MotorDriver: clock too slow for DShot pulse timing");
    end
    // synthesis translate_on

    always @(posedge clk) begin
        if (rst) begin
            esc_signal <= 0;
            busy <= 0;
            counter <= 0;
            packet <= 0;
            bit_index <= 0;
            gap <= 0;
        end else if (!busy) begin
            esc_signal <= 0;
            if (send) begin
                packet <= next_packet;
                bit_index <= 15;
                counter <= 0;
                gap <= 0;
                busy <= 1;
                esc_signal <= 1;
            end
        end else if (gap) begin
            esc_signal <= 0;
            if (counter == GAP_CYCLES - 1) begin
                counter <= 0;
                busy <= 0;
                gap <= 0;
            end else counter <= counter + 1'b1;
        end else if (counter == BIT_CYCLES - 1) begin
            counter <= 0;
            if (bit_index == 0) begin
                gap <= 1;
                esc_signal <= 0;
            end else begin
                bit_index <= bit_index - 1'b1;
                esc_signal <= 1;
            end
        end else begin
            counter <= counter + 1'b1;
            if (counter == (packet[bit_index] ? ONE_HIGH : ZERO_HIGH) - 1)
                esc_signal <= 0;
        end
    end
endmodule
