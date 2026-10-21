`timescale 1ns / 1ps
// Six-axis, small-angle complementary estimator. Sensor mounting: X forward,
// Y left, Z up. At rest the accelerometer reports +16384 on Z (+/-2g).
module ImuEstimator #(
    parameter integer CLK_FREQ = 100_000_000,
    parameter integer CALIBRATION_SAMPLES = 256,
    parameter integer TIMEOUT_MS = 100
) (
    input wire clk, rst, raw_valid, sensor_ready, sensor_fault,
    input wire signed [15:0] accel_x, accel_y, accel_z,
    input wire signed [15:0] gyro_x, gyro_y, gyro_z,
    output reg sample_valid, calibrated, fault,
    output wire valid,
    output reg signed [15:0] roll_cd, pitch_cd,
    output reg signed [15:0] roll_rate_cdps, pitch_rate_cdps, yaw_rate_cdps
);
    localparam integer TIMEOUT_CYCLES = (CLK_FREQ/1000)*TIMEOUT_MS;
    reg [31:0] age;
    integer count, sum_x, sum_y, sum_z, bias_x, bias_y, bias_z;
    reg signed [31:0] roll_q16, pitch_q16;
    integer ax, ay, az, gx, gy, gz, rx, ry, rz, gravity_squared;
    integer accel_roll, accel_pitch;
    reg stationary, gravity_ok;
    reg signed [63:0] predicted_roll, predicted_pitch;
    assign valid = sensor_ready && calibrated && !fault && age < TIMEOUT_CYCLES && !rst;

    always @* begin
        ax = accel_x; ay = accel_y; az = accel_z;
        gx = gyro_x; gy = gyro_y; gz = gyro_z;
        stationary = ax >= -1024 && ax <= 1024 && ay >= -1024 && ay <= 1024 &&
            az >= 15564 && az <= 17204 && gx >= -262 && gx <= 262 &&
            gy >= -262 && gy <= 262 && gz >= -262 && gz <= 262;
        // Limit components before squaring to prevent signed integer overflow.
        gravity_squared = (ax/4)*(ax/4) + (ay/4)*(ay/4) + (az/4)*(az/4);
        gravity_ok = az > 12000 && gravity_squared >= 10737418 && gravity_squared <= 24159191;
        // sin(theta) ~ theta near level; bounded to the prototype's small angles.
        accel_roll = ay * 5730 / 16384;
        accel_pitch = ax * 5730 / 16384;
        rx = (gx-bias_x)*100/131;
        ry = -(gy-bias_y)*100/131;
        rz = -(gz-bias_z)*100/131;
        // Extend BEFORE multiplication. Measured elapsed time includes skipped
        // polls, rather than assuming every read has a fixed sample period.
        predicted_roll = roll_q16;
        predicted_pitch = pitch_q16;
        if (raw_valid && calibrated) begin
            predicted_roll = roll_q16 + (($signed(rx)*64'sd65536)*($signed({1'b0,age})+1))/CLK_FREQ;
            predicted_pitch = pitch_q16 + (($signed(ry)*64'sd65536)*($signed({1'b0,age})+1))/CLK_FREQ;
            if (gravity_ok) begin
                predicted_roll = predicted_roll + (accel_roll*64'sd65536-predicted_roll)/256;
                predicted_pitch = predicted_pitch + (accel_pitch*64'sd65536-predicted_pitch)/256;
            end
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            sample_valid <= 0; calibrated <= 0; fault <= 0; age <= 0;
            count <= 0; sum_x <= 0; sum_y <= 0; sum_z <= 0;
            bias_x <= 0; bias_y <= 0; bias_z <= 0;
            roll_q16 <= 0; pitch_q16 <= 0; roll_cd <= 0; pitch_cd <= 0;
            roll_rate_cdps <= 0; pitch_rate_cdps <= 0; yaw_rate_cdps <= 0;
        end else begin
            sample_valid <= 0;
            if (age < TIMEOUT_CYCLES) age <= age + 1'b1;
            if (sensor_fault || (calibrated && (!sensor_ready || age >= TIMEOUT_CYCLES))) begin
                fault <= 1; calibrated <= 0;
            end
            if (raw_valid && sensor_ready && !sensor_fault && !fault &&
                (!calibrated || age < TIMEOUT_CYCLES)) begin
                age <= 0;
                if (!calibrated) begin
                    if (!stationary || (age >= TIMEOUT_CYCLES && count != 0)) begin
                        count <= 0; sum_x <= 0; sum_y <= 0; sum_z <= 0;
                    end else if (count == CALIBRATION_SAMPLES-1) begin
                        bias_x <= (sum_x+gx)/CALIBRATION_SAMPLES;
                        bias_y <= (sum_y+gy)/CALIBRATION_SAMPLES;
                        bias_z <= (sum_z+gz)/CALIBRATION_SAMPLES;
                        roll_q16 <= accel_roll*65536; pitch_q16 <= accel_pitch*65536;
                        roll_cd <= accel_roll; pitch_cd <= accel_pitch;
                        roll_rate_cdps <= 0; pitch_rate_cdps <= 0; yaw_rate_cdps <= 0;
                        calibrated <= 1; sample_valid <= 1;
                    end else begin
                        count <= count+1; sum_x <= sum_x+gx; sum_y <= sum_y+gy; sum_z <= sum_z+gz;
                    end
                end else if (gx >= 32700 || gx <= -32700 || gy >= 32700 || gy <= -32700 ||
                             gz >= 32700 || gz <= -32700 || ax >= 32700 || ax <= -32700 ||
                             ay >= 32700 || ay <= -32700 || az >= 32700 || az <= -32700 ||
                             rx > 32767 || rx < -32768 || ry > 32767 || ry < -32768 ||
                             rz > 32767 || rz < -32768 ||
                             predicted_roll > 35*100*64'sd65536 || predicted_roll < -35*100*64'sd65536 ||
                             predicted_pitch > 35*100*64'sd65536 || predicted_pitch < -35*100*64'sd65536) begin
                    fault <= 1; calibrated <= 0;
                end else begin
                    roll_q16 <= predicted_roll; pitch_q16 <= predicted_pitch;
                    roll_cd <= predicted_roll/65536; pitch_cd <= predicted_pitch/65536;
                    roll_rate_cdps <= rx; pitch_rate_cdps <= ry; yaw_rate_cdps <= rz;
                    sample_valid <= 1;
                end
            end
        end
    end
    // synthesis translate_off
    initial begin
        if (CLK_FREQ < 1000 || CALIBRATION_SAMPLES < 2 || CALIBRATION_SAMPLES > 4096 ||
            TIMEOUT_MS <= 0 || TIMEOUT_CYCLES < 2)
            $fatal(1, "ImuEstimator: invalid timing or calibration count");
    end
    // synthesis translate_on
endmodule
