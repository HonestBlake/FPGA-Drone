`timescale 1ns / 1ps

// Radio control mapping: one-based channels, verified on the transmitter.
// arm_request is a pilot request, not an arming state machine.
module RCMapper #(
    parameter integer ROLL_CHANNEL = 1,
    parameter integer PITCH_CHANNEL = 2,
    parameter integer THROTTLE_CHANNEL = 3,
    parameter integer YAW_CHANNEL = 4,
    parameter integer ARM_CHANNEL = 5,
    parameter integer MODE_CHANNEL = 6,
    parameter integer SWITCH_THRESHOLD = 1500
) (
    input wire [223:0] channels,
    input wire rc_valid,
    output wire [9:0] throttle,
    output wire signed [10:0] roll_command,
    output wire signed [10:0] pitch_command,
    output wire signed [10:0] yaw_command,
    output wire arm_request,
    output wire avoidance_enable
);
    // Limit analog channel values before subtracting their offset so inputs
    // outside 1000..2000 cannot produce commands outside the intended range.
    function [10:0] clamp_channel;
        input [15:0] value;
        begin
            if (value < 1000) clamp_channel = 11'd1000;
            else if (value > 2000) clamp_channel = 11'd2000;
            else clamp_channel = value[10:0];
        end
    endfunction

    // Channels are numbered 1..14; (channel - 1) * 16 locates the first bit
    // and +: 16 selects that channel's complete word from the packed bus.
    // Remove 1000 for throttle (0..1000), or 1500 for axes (-500..500).
    // $signed makes values below the center negative in the 11-bit outputs.
    // rc_valid gates every output to zero/false until a good packet arrives
    // and again if reception times out. This mapping has no clocked state.
    assign throttle = rc_valid ?
        clamp_channel(channels[(THROTTLE_CHANNEL-1)*16 +: 16]) - 11'd1000 : 10'd0;
    assign roll_command = rc_valid ?
        $signed(clamp_channel(channels[(ROLL_CHANNEL-1)*16 +: 16]) - 11'd1500) : 11'sd0;
    assign pitch_command = rc_valid ?
        $signed(clamp_channel(channels[(PITCH_CHANNEL-1)*16 +: 16]) - 11'd1500) : 11'sd0;
    assign yaw_command = rc_valid ?
        $signed(clamp_channel(channels[(YAW_CHANNEL-1)*16 +: 16]) - 11'd1500) : 11'sd0;
    // Switch channels use the raw value: at or above the threshold is true.
    assign arm_request = rc_valid && channels[(ARM_CHANNEL-1)*16 +: 16] >= SWITCH_THRESHOLD;
    assign avoidance_enable = rc_valid && channels[(MODE_CHANNEL-1)*16 +: 16] >= SWITCH_THRESHOLD;

    // synthesis translate_off
    initial begin
        if (ROLL_CHANNEL < 1 || ROLL_CHANNEL > 14 ||
            PITCH_CHANNEL < 1 || PITCH_CHANNEL > 14 ||
            THROTTLE_CHANNEL < 1 || THROTTLE_CHANNEL > 14 ||
            YAW_CHANNEL < 1 || YAW_CHANNEL > 14 ||
            ARM_CHANNEL < 1 || ARM_CHANNEL > 14 ||
            MODE_CHANNEL < 1 || MODE_CHANNEL > 14 ||
            SWITCH_THRESHOLD < 1000 || SWITCH_THRESHOLD > 2000)
            $fatal(1, "RCMapper: invalid channel number or switch threshold");
    end
    // synthesis translate_on
endmodule
