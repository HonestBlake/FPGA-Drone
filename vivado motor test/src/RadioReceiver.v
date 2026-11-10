`timescale 1ns / 1ps

// Radio subsystem. Child modules implement UART, iBUS decoding, and mapping.
// clk: Basys3 100 MHz; rst: synchronous active-high reset, assert before use.
// ibus_rx: idle-high, non-inverted, 3.3 V compatible servo iBUS input.
// Data path: serial RX -> UART bytes -> checked iBUS channels -> commands.
// All sequential stages use the same clk; UART bit timing is counted locally
// from this clock and does not depend on the flight controller's control_tick.
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
    // Internal byte interface: data is held between bytes; data_valid strobes
    // for one clk when a byte is ready. framing_error reports a low stop bit.
    wire [7:0] data;
    wire data_valid, framing_error;

    // Synchronize the asynchronous pin and sample the centers of UART bits.
    UartReceiver #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)) uart (
        .clk(clk), .rst(rst), .rx(ibus_rx), .data(data),
        .data_valid(data_valid), .framing_error(framing_error)
    );

    // Assemble 32-byte packets and publish all 14 channels together only
    // after a valid checksum. BYTE_TIMEOUT_US discards partial packets after
    // a long byte gap; TIMEOUT_MS clears rc_valid when good packets stop.
    IBusDecoder #(
        .CLK_FREQ(CLK_FREQ), .TIMEOUT_MS(TIMEOUT_MS),
        .BYTE_TIMEOUT_US(BYTE_TIMEOUT_US)
    ) decoder (
        .clk(clk), .rst(rst), .data(data), .data_valid(data_valid),
        .framing_error(framing_error), .channels(channels),
        .frame_valid(frame_valid), .rc_valid(rc_valid)
    );

    // Combinational mapping selects the configured one-based channels:
    // throttle is 0..1000, centered axes are -500..500, and switches become
    // booleans. When rc_valid is low, all mapped commands are zero/false.
    // Raw channels retain the last accepted values, so consumers must also
    // check rc_valid. frame_valid marks a new packet, not a persistent status.
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
