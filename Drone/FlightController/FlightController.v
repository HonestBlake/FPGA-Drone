`timescale 1ns / 1ps

// Prototype, small-angle PD stabilization and X-quad mixing. All data is in clk.
// Consumes calibrated roll/pitch and body rates from the IMU estimator.
module FlightController #(
    parameter integer CLK_FREQ = 100_000_000,
    parameter integer IMU_TIMEOUT_MS = 100,
    parameter integer CONTROL_TIMEOUT_MS = 10,
    parameter integer NPU_TIMEOUT_MS = 500,
    parameter integer ARM_MAX_THROTTLE = 0,
    parameter integer FORWARD_DEADBAND = 20,
    parameter integer MAX_PITCH_CD = 2000,
    parameter integer MAX_YAW_RATE_CDPS = 9000,
    parameter integer DIVERT_YAW_RATE_CDPS = 4500,
    parameter integer DIVERT_UP_THROTTLE = 100,
    // Gains are DShot counts per centidegree (or centidegree/s), scaled by 256.
    parameter integer ATTITUDE_KP = 32,
    parameter integer RATE_KD = 4,
    parameter integer YAW_KP = 8,
    parameter integer MAX_CORRECTION = 400
) (
    input wire clk,
    input wire rst,
    input wire control_tick,
    input wire [9:0] throttle,
    input wire signed [10:0] roll_command, // Observed for compatibility; ignored.
    input wire signed [10:0] pitch_command, // Positive = forward; negative ignored.
    input wire signed [10:0] yaw_command,   // Positive = turn right.
    input wire arm_request,
    input wire avoidance_enable,
    input wire frame_valid,
    input wire rc_valid,
    input wire motor_ready,
    input wire imu_sample_valid, // One clk pulse; atomically captures all five values.
    input wire imu_valid,        // Persistent estimate health.
    input wire imu_calibrated,
    input wire imu_fault,
    input wire signed [15:0] imu_roll_cd,
    input wire signed [15:0] imu_pitch_cd,
    input wire signed [15:0] imu_roll_rate_cdps,
    input wire signed [15:0] imu_pitch_rate_cdps,
    input wire signed [15:0] imu_yaw_rate_cdps,
    input wire npu_available,
    input wire npu_action_valid, // One clk pulse, even when repeating an action.
    input wire [2:0] npu_action,
    output wire available,       // Implementation exists; not an arming permission.
    output wire command_valid,   // Healthy inputs and a running control clock.
    output wire imu_fresh,
    output wire npu_fresh,
    output wire motors_armed,
    output reg avoidance_active,
    output reg [2:0] applied_action,
    output reg signed [15:0] pitch_setpoint_cd,
    output reg signed [15:0] yaw_rate_setpoint_cdps,
    output reg [9:0] thrust_setpoint,
    output wire [10:0] front_right_throttle,
    output wire [10:0] front_left_throttle,
    output wire [10:0] back_right_throttle,
    output wire [10:0] back_left_throttle
);
    localparam [2:0] CONTINUE = 3'd0, STOP = 3'd1, DIVERT_UP = 3'd2,
                     DIVERT_LEFT = 3'd3, DIVERT_RIGHT = 3'd4;
    // Split multiplication to avoid overflowing 32-bit integers at 100 MHz.
    localparam integer IMU_CYCLES = (CLK_FREQ / 1000) * IMU_TIMEOUT_MS +
        ((CLK_FREQ % 1000) * IMU_TIMEOUT_MS + 999) / 1000;
    localparam integer CONTROL_CYCLES = (CLK_FREQ / 1000) * CONTROL_TIMEOUT_MS +
        ((CLK_FREQ % 1000) * CONTROL_TIMEOUT_MS + 999) / 1000;
    localparam integer NPU_CYCLES = (CLK_FREQ / 1000) * NPU_TIMEOUT_MS +
        ((CLK_FREQ % 1000) * NPU_TIMEOUT_MS + 999) / 1000;
    localparam integer IMU_WIDTH = (IMU_CYCLES > 0) ? $clog2(IMU_CYCLES + 1) : 1;
    localparam integer CONTROL_WIDTH = (CONTROL_CYCLES > 0) ? $clog2(CONTROL_CYCLES + 1) : 1;
    localparam integer NPU_WIDTH = (NPU_CYCLES > 0) ? $clog2(NPU_CYCLES + 1) : 1;
    reg [IMU_WIDTH-1:0] imu_age;
    reg [CONTROL_WIDTH-1:0] control_age;
    reg [NPU_WIDTH-1:0] npu_age;
    reg imu_seen, control_seen, npu_seen, armed_state, arm_permit;
    reg signed [15:0] roll_cd, pitch_cd, roll_rate, pitch_rate, yaw_rate;
    reg [2:0] action_latched;
    reg [10:0] motor_fr, motor_fl, motor_br, motor_bl;

    assign available = !rst;
    assign imu_fresh = !rst && imu_seen && imu_age < IMU_CYCLES &&
                       imu_valid && imu_calibrated && !imu_fault;
    assign npu_fresh = !rst && npu_available && npu_seen && npu_age < NPU_CYCLES;
    assign command_valid = !rst && rc_valid && imu_fresh && motor_ready &&
                           control_seen && control_age < CONTROL_CYCLES;
    assign motors_armed = armed_state && command_valid && arm_request;
    // Gate immediately on a health loss, without waiting for another control tick.
    assign front_right_throttle = motors_armed ? motor_fr : 11'd0;
    assign front_left_throttle  = motors_armed ? motor_fl : 11'd0;
    assign back_right_throttle  = motors_armed ? motor_br : 11'd0;
    assign back_left_throttle   = motors_armed ? motor_bl : 11'd0;

    always @(posedge clk) begin
        if (rst) begin
            imu_age <= IMU_CYCLES; control_age <= CONTROL_CYCLES; npu_age <= NPU_CYCLES;
            imu_seen <= 0; control_seen <= 0; npu_seen <= 0;
            roll_cd <= 0; pitch_cd <= 0; roll_rate <= 0; pitch_rate <= 0; yaw_rate <= 0;
            action_latched <= CONTINUE;
        end else begin
            if (!imu_valid || !imu_calibrated || imu_fault) begin
                imu_seen <= 0; imu_age <= IMU_CYCLES;
            end else if (imu_sample_valid) begin
                imu_seen <= 1; imu_age <= 0;
                roll_cd <= imu_roll_cd; pitch_cd <= imu_pitch_cd;
                roll_rate <= imu_roll_rate_cdps; pitch_rate <= imu_pitch_rate_cdps;
                yaw_rate <= imu_yaw_rate_cdps;
            end else if (imu_age < IMU_CYCLES) imu_age <= imu_age + 1'b1;

            if (control_tick) begin control_seen <= 1; control_age <= 0; end
            else if (control_age < CONTROL_CYCLES) control_age <= control_age + 1'b1;

            if (!npu_available) begin npu_seen <= 0; npu_age <= NPU_CYCLES; end
            else if (npu_action_valid) begin
                npu_seen <= 1; npu_age <= 0;
                // Unknown actions conservatively cancel forward motion.
                action_latched <= (npu_action <= DIVERT_RIGHT) ? npu_action : STOP;
            end else if (npu_age < NPU_CYCLES) npu_age <= npu_age + 1'b1;
        end
    end

    // Require a healthy arm-OFF radio packet before an arm-ON packet at idle.
    // A rejected arm request or any health failure consumes that permission.
    always @(posedge clk) begin
        if (rst || !command_valid) begin armed_state <= 0; arm_permit <= 0; end
        else if (!arm_request) begin
            armed_state <= 0;
            if (frame_valid) arm_permit <= 1;
        end else if (frame_valid && !armed_state) begin
            armed_state <= arm_permit && (throttle <= ARM_MAX_THROTTLE);
            arm_permit <= 0;
        end
    end

    function integer clamp_axis;
        input integer value;
        begin
            if (value < -500) clamp_axis = -500;
            else if (value > 500) clamp_axis = 500;
            else clamp_axis = value;
        end
    endfunction
    function integer clamp_correction;
        input integer value;
        begin
            if (value < -MAX_CORRECTION) clamp_correction = -MAX_CORRECTION;
            else if (value > MAX_CORRECTION) clamp_correction = MAX_CORRECTION;
            else clamp_correction = value;
        end
    endfunction
    function [10:0] clamp_motor;
        input integer value;
        begin
            if (value < 48) clamp_motor = 11'd48;
            else if (value > 2047) clamp_motor = 11'd2047;
            else clamp_motor = value[10:0];
        end
    endfunction

    integer next_thrust, next_pitch, next_yaw, base;
    integer roll_correction, pitch_correction, yaw_correction;
    reg next_avoidance;
    reg [2:0] next_action;
    always @* begin
        next_thrust = (throttle > 1000) ? 1000 : throttle;
        next_pitch = 0;
        if ($signed(pitch_command) > FORWARD_DEADBAND)
            next_pitch = -(clamp_axis($signed(pitch_command)) * MAX_PITCH_CD) / 500;
        next_yaw = (clamp_axis($signed(yaw_command)) * MAX_YAW_RATE_CDPS) / 500;
        next_avoidance = 0;
        next_action = CONTINUE;
        // Gate with ORIGINAL pilot intent, so STOP/turning does not self-cancel.
        if ($signed(pitch_command) > FORWARD_DEADBAND && avoidance_enable && npu_available) begin
            next_action = npu_fresh ? action_latched : STOP;
            case (next_action)
                STOP: begin next_pitch = 0; next_yaw = 0; next_avoidance = 1; end
                DIVERT_UP: begin
                    next_pitch = 0; next_yaw = 0; next_avoidance = 1;
                    // Do not start motors from a zero-thrust pilot request.
                    if (next_thrust > 0) next_thrust = next_thrust + DIVERT_UP_THROTTLE;
                    if (next_thrust > 1000) next_thrust = 1000;
                end
                DIVERT_LEFT: begin
                    next_pitch = 0; next_yaw = -DIVERT_YAW_RATE_CDPS; next_avoidance = 1;
                end
                DIVERT_RIGHT: begin
                    next_pitch = 0; next_yaw = DIVERT_YAW_RATE_CDPS; next_avoidance = 1;
                end
                default: begin end // CONTINUE passes the pilot commands.
            endcase
        end
        // Small-angle PD roll/pitch controller; proportional yaw-rate feedback.
        roll_correction = clamp_correction((-roll_cd * ATTITUDE_KP - roll_rate * RATE_KD) / 256);
        pitch_correction = clamp_correction(((next_pitch - pitch_cd) * ATTITUDE_KP - pitch_rate * RATE_KD) / 256);
        yaw_correction = clamp_correction(((next_yaw - yaw_rate) * YAW_KP) / 256);
        base = 48 + (next_thrust * 1999) / 1000;
    end

    always @(posedge clk) begin
        if (rst || !motors_armed) begin
            motor_fr <= 0; motor_fl <= 0; motor_br <= 0; motor_bl <= 0;
            avoidance_active <= 0; applied_action <= CONTINUE;
            pitch_setpoint_cd <= 0; yaw_rate_setpoint_cdps <= 0; thrust_setpoint <= 0;
        end else if (control_tick) begin
            avoidance_active <= next_avoidance; applied_action <= next_action;
            pitch_setpoint_cd <= next_pitch;
            yaw_rate_setpoint_cdps <= next_yaw;
            thrust_setpoint <= next_thrust;
            if (next_thrust == 0) begin
                motor_fr <= 0; motor_fl <= 0; motor_br <= 0; motor_bl <= 0;
            end else begin
                // FRD body axes: +roll right-down, +pitch nose-up, +yaw right.
                // Assumed spin viewed from above: FR/BL CCW, FL/BR CW.
                motor_fr <= clamp_motor(base - roll_correction + pitch_correction + yaw_correction);
                motor_fl <= clamp_motor(base + roll_correction + pitch_correction - yaw_correction);
                motor_br <= clamp_motor(base - roll_correction - pitch_correction - yaw_correction);
                motor_bl <= clamp_motor(base + roll_correction - pitch_correction + yaw_correction);
            end
        end
    end

    // synthesis translate_off
    initial begin
        if (CLK_FREQ <= 0 || IMU_CYCLES < 2 || CONTROL_CYCLES < 2 || NPU_CYCLES < 2 ||
            IMU_TIMEOUT_MS <= 0 || CONTROL_TIMEOUT_MS <= 0 || NPU_TIMEOUT_MS <= 0 ||
            ARM_MAX_THROTTLE < 0 || ARM_MAX_THROTTLE > 1000 ||
            FORWARD_DEADBAND < 0 || FORWARD_DEADBAND >= 500 ||
            MAX_PITCH_CD < 0 || MAX_PITCH_CD > 4500 ||
            MAX_YAW_RATE_CDPS < 0 || MAX_YAW_RATE_CDPS > 18000 ||
            DIVERT_YAW_RATE_CDPS < 0 || DIVERT_YAW_RATE_CDPS > MAX_YAW_RATE_CDPS ||
            DIVERT_UP_THROTTLE < 0 || DIVERT_UP_THROTTLE > 1000 ||
            ATTITUDE_KP < 0 || ATTITUDE_KP > 4096 || RATE_KD < 0 || RATE_KD > 4096 ||
            YAW_KP < 0 || YAW_KP > 4096 || MAX_CORRECTION < 0 || MAX_CORRECTION > 1999)
            $fatal(1, "FlightController: invalid timeout, limit, or gain");
    end
    // synthesis translate_on
endmodule
