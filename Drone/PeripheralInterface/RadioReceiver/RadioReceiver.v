`timescale 1ns / 1ps

// Radio subsystem. Child modules implement UART, iBUS decoding, and mapping.
// clk: Basys3 100 MHz; rst: synchronous active-high reset, assert before use.
// ibus_rx: idle-high, non-inverted, 3.3 V compatible servo iBUS input.
module RadioReceiver #(
    parameter integer CLK_FREQ = 100_000_000,
    parameter integer BAUD_RATE = 115_200,
    parameter integer TIMEOUT_MS = 100,
    parameter integer BYTE_TIMEOUT_US = 500,
    parameter integer ROLL_CHANNEL = 1,
    parameter integer PITCH_CHANNEL = 2,
    parameter integer THROTTLE_CHANNEL = 3,
    parameter integer YAW_CHANNEL = 4,
    parameter integer ARM_CHANNEL = 5,
    parameter integer MODE_CHANNEL = 6,
    parameter integer SWITCH_THRESHOLD = 1500
) (
    input wire clk,
    input wire rst,
    input wire ibus_rx,
    output wire [9:0] throttle,
    output wire signed [10:0] roll_command,
    output wire signed [10:0] pitch_command,
    output wire signed [10:0] yaw_command,
    output wire arm_request,
    output wire avoidance_enable,
    output wire frame_valid, // One clk pulse when channels and commands update.
    output wire rc_valid,    // Recent checksum-valid packet, not RF link status.
    output wire [223:0] channels
);
    wire [7:0] data;
    wire data_valid, framing_error;

    UartReceiver #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)) uart (
        .clk(clk), .rst(rst), .rx(ibus_rx), .data(data),
        .data_valid(data_valid), .framing_error(framing_error)
    );

    IBusDecoder #(
        .CLK_FREQ(CLK_FREQ), .TIMEOUT_MS(TIMEOUT_MS),
        .BYTE_TIMEOUT_US(BYTE_TIMEOUT_US)
    ) decoder (
        .clk(clk), .rst(rst), .data(data), .data_valid(data_valid),
        .framing_error(framing_error), .channels(channels),
        .frame_valid(frame_valid), .rc_valid(rc_valid)
    );

    RCMapper #(
        .ROLL_CHANNEL(ROLL_CHANNEL), .PITCH_CHANNEL(PITCH_CHANNEL),
        .THROTTLE_CHANNEL(THROTTLE_CHANNEL), .YAW_CHANNEL(YAW_CHANNEL),
        .ARM_CHANNEL(ARM_CHANNEL), .MODE_CHANNEL(MODE_CHANNEL),
        .SWITCH_THRESHOLD(SWITCH_THRESHOLD)
    ) mapper (
        .channels(channels), .rc_valid(rc_valid), .throttle(throttle),
        .roll_command(roll_command), .pitch_command(pitch_command),
        .yaw_command(yaw_command), .arm_request(arm_request),
        .avoidance_enable(avoidance_enable)
    );
endmodule
