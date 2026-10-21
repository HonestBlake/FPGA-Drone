`include "../../../Drone/PeripheralInterface/RadioReceiver/RCMapper.v"
`timescale 1ns / 1ps
module RCMapper_tb;
    reg [223:0] channels = 0;
    reg rc_valid = 0;
    wire [9:0] throttle;
    wire signed [10:0] roll_command, pitch_command, yaw_command;
    wire arm_request, avoidance_enable;
    integer value, bounded, ch;
    RCMapper dut (
        .channels(channels), .rc_valid(rc_valid), .throttle(throttle),
        .roll_command(roll_command), .pitch_command(pitch_command),
        .yaw_command(yaw_command), .arm_request(arm_request),
        .avoidance_enable(avoidance_enable)
    );
    wire [9:0] alternate_throttle;
    wire signed [10:0] alternate_roll;
    wire alternate_arm;
    RCMapper #(.ROLL_CHANNEL(14), .THROTTLE_CHANNEL(7),
               .ARM_CHANNEL(8), .SWITCH_THRESHOLD(1750)) alternate (
        .channels(channels), .rc_valid(rc_valid), .throttle(alternate_throttle),
        .roll_command(alternate_roll), .pitch_command(), .yaw_command(),
        .arm_request(alternate_arm), .avoidance_enable()
    );
    initial begin
        $dumpfile("RCMapper.vcd");
        $dumpvars(0, rc_valid, throttle, roll_command, pitch_command, yaw_command,
                  arm_request, avoidance_enable, alternate_throttle, alternate_roll, alternate_arm);
        #1;
        if ({throttle, roll_command, pitch_command, yaw_command, arm_request,
             avoidance_enable} !== 45'b0) $fatal(1, "Invalid commands not zero");
        rc_valid = 1;
        for (value = 0; value < 65536; value = value + 1) begin
            for (ch = 0; ch < 14; ch = ch + 1) channels[ch*16 +: 16] = value;
            #1;
            bounded = value < 1000 ? 1000 : (value > 2000 ? 2000 : value);
            if (throttle !== bounded-1000 || $signed(roll_command) !== bounded-1500 ||
                $signed(pitch_command) !== bounded-1500 || $signed(yaw_command) !== bounded-1500 ||
                arm_request !== (value >= 1500) || avoidance_enable !== (value >= 1500) ||
                alternate_arm !== (value >= 1750))
                $fatal(1, "Mapping failed at raw value %0d", value);
        end
        channels[13*16 +: 16] = 1750;
        channels[6*16 +: 16] = 1234;
        channels[7*16 +: 16] = 1749;
        #1;
        if (alternate_roll !== 11'sd250 || alternate_throttle !== 234 || alternate_arm !== 0)
            $fatal(1, "Non-default channel routing failed");
        rc_valid = 0; #1;
        if ({throttle, roll_command, pitch_command, yaw_command, arm_request,
             avoidance_enable} !== 45'b0 || alternate_throttle !== 0 ||
             alternate_roll !== 0 || alternate_arm !== 0)
            $fatal(1, "Validity gating failed");
        $display("PASS: RCMapper all 65536 values, channel routing, threshold, invalid gating");
        $finish;
    end
endmodule
