`include "../../Drone/FlightController/FlightController.v"
`timescale 1ns / 1ps
module FlightController_tb;
    reg clk = 0, rst = 1;
    reg [9:0] throttle = 0;
    reg signed [10:0] roll_command = 0, pitch_command = 0, yaw_command = 0;
    reg arm_request = 0, avoidance_enable = 0, frame_valid = 0, rc_valid = 0;
    reg motor_ready = 0, imu_valid = 0, imu_calibrated = 0, imu_fault = 0;
    reg signed [15:0] imu_roll = 0, imu_pitch = 0, imu_p = 0, imu_q = 0, imu_r = 0;
    reg npu_available = 0, npu_action_valid = 0;
    reg [2:0] npu_action = 0;
    reg run_ticks = 1, stream_imu = 0, manual_sample = 0;
    integer cycle = 0;
    wire control_tick = run_ticks && cycle % 4 == 0;
    wire sample_valid = manual_sample || (stream_imu && cycle % 7 == 0);
    wire available, motors_armed, command_valid, imu_fresh, npu_fresh, avoidance_active;
    wire [2:0] applied_action;
    wire signed [15:0] pitch_sp, yaw_sp;
    wire [9:0] thrust_sp;
    wire [10:0] fr, fl, br, bl;
    reg [43:0] held_motors;
    always #5 clk = ~clk;
    always @(negedge clk) cycle = cycle + 1;
    // Small clock-frequency parameter makes the watchdog boundaries quick to test.
    FlightController #(.CLK_FREQ(10_000), .IMU_TIMEOUT_MS(20),
        .CONTROL_TIMEOUT_MS(5), .NPU_TIMEOUT_MS(10)) dut (
        .clk(clk), .rst(rst), .control_tick(control_tick), .throttle(throttle),
        .roll_command(roll_command), .pitch_command(pitch_command), .yaw_command(yaw_command),
        .arm_request(arm_request), .avoidance_enable(avoidance_enable),
        .frame_valid(frame_valid), .rc_valid(rc_valid), .motor_ready(motor_ready),
        .imu_sample_valid(sample_valid), .imu_valid(imu_valid),
        .imu_calibrated(imu_calibrated), .imu_fault(imu_fault),
        .imu_roll_cd(imu_roll), .imu_pitch_cd(imu_pitch),
        .imu_roll_rate_cdps(imu_p), .imu_pitch_rate_cdps(imu_q), .imu_yaw_rate_cdps(imu_r),
        .npu_available(npu_available), .npu_action_valid(npu_action_valid), .npu_action(npu_action),
        .available(available), .command_valid(command_valid), .motors_armed(motors_armed),
        .imu_fresh(imu_fresh), .npu_fresh(npu_fresh), .avoidance_active(avoidance_active),
        .applied_action(applied_action), .pitch_setpoint_cd(pitch_sp),
        .yaw_rate_setpoint_cdps(yaw_sp), .thrust_setpoint(thrust_sp),
        .front_right_throttle(fr), .front_left_throttle(fl),
        .back_right_throttle(br), .back_left_throttle(bl)
    );
    task step;
        input integer count;
        begin repeat (count) @(negedge clk); #1; end
    endtask
    task radio_frame;
        input arm;
        begin
            arm_request = arm; frame_valid = 1;
            step(1); frame_valid = 0; step(6);
        end
    endtask
    task rearm;
        begin
            throttle = 0;
            radio_frame(0); radio_frame(1);
            if (!motors_armed) $fatal(1, "Healthy idle arm sequence rejected");
            if ({fr, fl, br, bl} !== 44'd0) $fatal(1, "Idle request spun motors");
            throttle = 500; step(8);
        end
    endtask
    task action;
        input [2:0] value;
        begin
            npu_action = value; npu_action_valid = 1;
            step(1); npu_action_valid = 0; step(8);
        end
    endtask
    task check_motors;
        input [10:0] a, b, c, d;
        begin
            if ({fr, fl, br, bl} !== {a, b, c, d})
                $fatal(1, "Motor mix %0d %0d %0d %0d, expected %0d %0d %0d %0d", fr, fl, br, bl, a, b, c, d);
        end
    endtask
    task check_stop;
        begin
            if (motors_armed || {fr, fl, br, bl} !== 44'd0)
                $fatal(1, "Interlock did not stop motors");
        end
    endtask
    // Independently ensure every published motor value is stop or ordinary DShot.
    always @(posedge clk) begin
        #2;
        if ((fr > 0 && fr < 48) || (fl > 0 && fl < 48) ||
            (br > 0 && br < 48) || (bl > 0 && bl < 48))
            $fatal(1, "Reserved ESC command emitted");
    end
    initial begin
        $dumpfile("FlightController.vcd"); $dumpvars(0, FlightController_tb);
        step(2); check_stop;
        if (available || command_valid || imu_fresh || npu_fresh) $fatal(1, "Reset status");
        rst = 0; rc_valid = 1;
        radio_frame(0); radio_frame(1); check_stop;
        imu_valid = 1; stream_imu = 1;
        step(10); check_stop; // Uncalibrated IMU and ESC not ready.
        imu_calibrated = 1; step(10); radio_frame(0); radio_frame(1); check_stop;
        motor_ready = 1; step(8); radio_frame(1); check_stop; // No automatic arm.
        throttle = 300; radio_frame(0); radio_frame(1); check_stop;
        throttle = 0; radio_frame(1); check_stop; // Rejected high-thrust arm is consumed.
        rearm; check_motors(1047, 1047, 1047, 1047);
        roll_command = -500; step(8); check_motors(1047, 1047, 1047, 1047);
        roll_command = 500; step(8); check_motors(1047, 1047, 1047, 1047);

        pitch_command = 500; step(8);
        if (pitch_sp !== -16'sd2000) $fatal(1, "Forward pitch sign/range");
        check_motors(797, 797, 1297, 1297);
        pitch_command = -500; step(8);
        if (pitch_sp !== 0) $fatal(1, "Reverse pitch accepted");
        check_motors(1047, 1047, 1047, 1047);
        pitch_command = 20; step(8);
        if (pitch_sp !== 0) $fatal(1, "Pitch deadband");
        yaw_command = 250; step(8); check_motors(1187, 907, 907, 1187);
        yaw_command = -250; step(8); check_motors(907, 1187, 1187, 907);
        yaw_command = 0; pitch_command = 0;

        imu_roll = 500; step(12); check_motors(1109, 985, 1109, 985);
        imu_roll = 0; imu_pitch = 500; step(12); check_motors(985, 985, 1109, 1109);
        imu_pitch = 0; imu_p = 1024; step(12); check_motors(1063, 1031, 1063, 1031);
        imu_p = 0; imu_q = 1024; step(12); check_motors(1031, 1031, 1063, 1063);
        imu_q = 0; imu_r = 1024; step(12); check_motors(1015, 1079, 1079, 1015);
        imu_r = 0; step(12);
        // Samples are latched atomically; bus changes alone must not alter mixing.
        stream_imu = 0; held_motors = {fr, fl, br, bl};
        imu_roll = 2000; imu_pitch = -2000; step(12);
        if ({fr, fl, br, bl} !== held_motors) $fatal(1, "IMU bus used without sample strobe");
        imu_roll = 0; imu_pitch = 0; stream_imu = 1; step(12);

        npu_available = 1; avoidance_enable = 1; pitch_command = 500; yaw_command = 250;
        step(8);
        if (!avoidance_active || pitch_sp !== 0 || yaw_sp !== 0 || thrust_sp !== 500)
            $fatal(1, "Missing available-NPU result must cancel forward command");
        action(0); check_motors(937, 657, 1157, 1437);
        if (avoidance_active || pitch_sp !== -2000 || yaw_sp !== 4500) $fatal(1, "CONTINUE");
        action(1); check_motors(1047, 1047, 1047, 1047);
        if (!motors_armed || !avoidance_active || pitch_sp !== 0 || thrust_sp !== 500)
            $fatal(1, "STOP must keep thrust and stabilization");
        action(2); check_motors(1247, 1247, 1247, 1247);
        if (thrust_sp !== 600 || pitch_sp !== 0) $fatal(1, "UP");
        action(3); check_motors(907, 1187, 1187, 907);
        if (yaw_sp !== -4500 || pitch_sp !== 0) $fatal(1, "LEFT");
        action(4); check_motors(1187, 907, 907, 1187);
        if (yaw_sp !== 4500 || pitch_sp !== 0) $fatal(1, "RIGHT");
        action(7);
        if (applied_action !== 1 || !avoidance_active) $fatal(1, "Invalid action should STOP");
        action(3); pitch_command = 0; step(8);
        if (avoidance_active || yaw_sp !== 4500 || thrust_sp !== 500) $fatal(1, "NPU used without forward intent");
        pitch_command = -500; step(8);
        if (avoidance_active || pitch_sp !== 0) $fatal(1, "NPU used for reverse input");
        pitch_command = 500; avoidance_enable = 0; step(8);
        if (avoidance_active || pitch_sp !== -2000) $fatal(1, "Mode switch did not bypass override");
        avoidance_enable = 1; npu_available = 0; step(8);
        if (avoidance_active || pitch_sp !== -2000 || npu_fresh) $fatal(1, "Unavailable NPU blocked manual control");
        npu_available = 1; action(0);
        wait (dut.npu_age == 99); #1;
        if (!npu_fresh) $fatal(1, "NPU timeout early");
        @(posedge clk); #1;
        if (npu_fresh) $fatal(1, "NPU timeout late");
        step(8);
        if (applied_action !== 1 || pitch_sp !== 0 || !motors_armed) $fatal(1, "Stale NPU policy");
        action(0);
        if (pitch_sp !== -2000 || !npu_fresh) $fatal(1, "NPU fresh-result recovery");

        action(2); throttle = 950; step(8);
        if (thrust_sp !== 1000) $fatal(1, "UP thrust not capped");
        throttle = 0; step(8); check_motors(0, 0, 0, 0);
        if (thrust_sp !== 0) $fatal(1, "UP overrode idle thrust");
        npu_available = 0; pitch_command = 0; yaw_command = 0;
        throttle = 1023; step(8); check_motors(2047, 2047, 2047, 2047);
        throttle = 1; imu_roll = 32767; imu_pitch = -32768; imu_r = -32768;
        step(12); check_motors(1249, 48, 48, 48);
        throttle = 1000; step(8); check_motors(2047, 1647, 1647, 1647);
        imu_roll = 0; imu_pitch = 0; imu_r = 0; throttle = 500; step(12);

        // Commands hold between control ticks, even if pilot input changes.
        run_ticks = 0; step(1); held_motors = {fr, fl, br, bl};
        throttle = 700; step(10);
        if ({fr, fl, br, bl} !== held_motors) $fatal(1, "Motor update outside control tick");
        wait (dut.control_age == 49); #1;
        if (!command_valid) $fatal(1, "Control timeout early");
        @(posedge clk); #1; check_stop;
        if (command_valid) $fatal(1, "Control timeout late");
        run_ticks = 1; step(8); radio_frame(1); check_stop; rearm;

        stream_imu = 0; manual_sample = 1; step(1); manual_sample = 0;
        wait (dut.imu_age == 199); #1;
        if (!imu_fresh) $fatal(1, "IMU timeout early");
        @(posedge clk); #1; check_stop;
        if (imu_fresh || command_valid) $fatal(1, "IMU timeout late");
        stream_imu = 1; step(12); radio_frame(1); check_stop; rearm;
        imu_fault = 1; step(1); check_stop;
        imu_fault = 0; step(12); radio_frame(1); check_stop; rearm;
        imu_valid = 0; step(1); check_stop;
        imu_valid = 1; step(12); rearm;
        imu_calibrated = 0; step(1); check_stop;
        imu_calibrated = 1; step(12); rearm;
        motor_ready = 0; step(1); check_stop;
        motor_ready = 1; step(12); radio_frame(1); check_stop; rearm;
        rc_valid = 0; step(1); check_stop;
        rc_valid = 1; step(12); radio_frame(1); check_stop; rearm;
        arm_request = 0; step(1); check_stop;
        rearm; rst = 1; step(1); check_stop;
        if (available || command_valid || imu_fresh || npu_fresh) $fatal(1, "Active reset status");
        $display("PASS: stabilization, mixing, arming, five NPU actions, gating, saturation, watchdogs and recovery");
        $finish;
    end
    initial begin #100000; $fatal(1, "FlightController test watchdog"); end
endmodule
