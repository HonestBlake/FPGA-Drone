`timescale 1ns / 1ps

// Aggregates the radio receiver, ICM-20948 IMU, depth array, and four-motor ESC interface.
module PeripheralInterface #(
    parameter integer CLK_FREQ = 100_000_000,
    parameter integer IMU_SPI_CLK_DIV = (CLK_FREQ + 9_999_999) / 10_000_000,
    parameter integer IMU_POWERUP_CYCLES = (CLK_FREQ/1000)*100,
    parameter integer IMU_RESET_CYCLES = (CLK_FREQ/1000)*100,
    parameter integer IMU_WAKE_CYCLES = (CLK_FREQ/1000)*35,
    parameter integer IMU_POLL_HZ = 500,
    parameter integer IMU_CALIBRATION_SAMPLES = 256,
    parameter integer IMU_TIMEOUT_MS = 100,
    parameter integer BAUD_RATE = 115_200,
    parameter integer TIMEOUT_MS = 100,
    parameter integer BYTE_TIMEOUT_US = 500,
    parameter integer ROLL_CHANNEL = 1,
    parameter integer PITCH_CHANNEL = 2,
    parameter integer THROTTLE_CHANNEL = 3,
    parameter integer YAW_CHANNEL = 4,
    parameter integer ARM_CHANNEL = 5,
    parameter integer MODE_CHANNEL = 6,
    parameter integer SWITCH_THRESHOLD = 1500,
    parameter integer LIDAR_STARTUP_MS = 1000,
    parameter integer LIDAR_SETTLE_MS = 5000,
    parameter integer LIDAR_POLL_MS = 100,
    parameter integer LIDAR_RESPONSE_TIMEOUT_MS = 8000,
    parameter integer LIDAR_BYTE_TIMEOUT_MS = 500,
    parameter integer MOTOR_DSHOT_RATE = 300_000,
    parameter integer MOTOR_UPDATE_HZ = 1_000,
    parameter integer MOTOR_STARTUP_MS = 1_000
) (
    input wire clk,
    input wire rst,
    input wire imu_spi_miso,
    output wire imu_spi_cs_n, imu_spi_sclk, imu_spi_mosi,
    output wire imu_raw_valid, imu_busy, imu_init_done, imu_detected,
    output wire imu_sensor_sample_valid, imu_sensor_valid, imu_sensor_calibrated, imu_sensor_fault,
    output wire signed [15:0] imu_accel_x, imu_accel_y, imu_accel_z,
    output wire signed [15:0] imu_gyro_x, imu_gyro_y, imu_gyro_z,
    output wire signed [15:0] imu_sensor_roll_cd, imu_sensor_pitch_cd,
    output wire signed [15:0] imu_sensor_roll_rate_cdps, imu_sensor_pitch_rate_cdps, imu_sensor_yaw_rate_cdps,
    input wire ibus_rx,
    output wire [9:0] throttle,
    output wire signed [10:0] roll_command,
    output wire signed [10:0] pitch_command,
    output wire signed [10:0] yaw_command,
    output wire arm_request,
    output wire avoidance_enable,
    output wire frame_valid, // One clk pulse when channels and commands update.
    output wire rc_valid,    // Recent checksum-valid packet, not RF link status.
    output wire [223:0] channels,
    input wire lidar_rx,
    output wire lidar_tx,
    input wire [5:0] lidar_read_address,
    output wire [15:0] lidar_distance_mm,
    output wire lidar_image_valid,
    output wire lidar_frame_valid,
    output wire [31:0] lidar_frame_count,
    output wire lidar_initialized,
    output wire lidar_error,
    input wire motors_armed,
    input wire motor_command_valid,
    input wire [10:0] front_right_throttle,
    input wire [10:0] front_left_throttle,
    input wire [10:0] back_right_throttle,
    input wire [10:0] back_left_throttle,
    output wire esc_front_right,
    output wire esc_front_left,
    output wire esc_back_right,
    output wire esc_back_left,
    output wire motor_ready,
    output wire motor_busy
);
    IMU #(
        .CLK_FREQ(CLK_FREQ),
        .SPI_CLK_DIV(IMU_SPI_CLK_DIV),
        .POWERUP_CYCLES(IMU_POWERUP_CYCLES),
        .RESET_CYCLES(IMU_RESET_CYCLES),
        .WAKE_CYCLES(IMU_WAKE_CYCLES),
        .POLL_HZ(IMU_POLL_HZ),
        .CALIBRATION_SAMPLES(IMU_CALIBRATION_SAMPLES),
        .TIMEOUT_MS(IMU_TIMEOUT_MS)
    ) imu (
        .clk(clk), .rst(rst),
        .spi_miso(imu_spi_miso),
        .spi_cs_n(imu_spi_cs_n),
        .spi_sclk(imu_spi_sclk),
        .spi_mosi(imu_spi_mosi),
        .raw_valid(imu_raw_valid),
        .busy(imu_busy),
        .init_done(imu_init_done),
        .detected(imu_detected),
        .accel_x(imu_accel_x),
        .accel_y(imu_accel_y),
        .accel_z(imu_accel_z),
        .gyro_x(imu_gyro_x),
        .gyro_y(imu_gyro_y),
        .gyro_z(imu_gyro_z),
        .sample_valid(imu_sensor_sample_valid),
        .valid(imu_sensor_valid),
        .calibrated(imu_sensor_calibrated),
        .fault(imu_sensor_fault),
        .roll_cd(imu_sensor_roll_cd),
        .pitch_cd(imu_sensor_pitch_cd),
        .roll_rate_cdps(imu_sensor_roll_rate_cdps),
        .pitch_rate_cdps(imu_sensor_pitch_rate_cdps),
        .yaw_rate_cdps(imu_sensor_yaw_rate_cdps)
    );

    RadioReceiver #(
        .CLK_FREQ(CLK_FREQ),
        .BAUD_RATE(BAUD_RATE),
        .TIMEOUT_MS(TIMEOUT_MS),
        .BYTE_TIMEOUT_US(BYTE_TIMEOUT_US),
        .ROLL_CHANNEL(ROLL_CHANNEL),
        .PITCH_CHANNEL(PITCH_CHANNEL),
        .THROTTLE_CHANNEL(THROTTLE_CHANNEL),
        .YAW_CHANNEL(YAW_CHANNEL),
        .ARM_CHANNEL(ARM_CHANNEL),
        .MODE_CHANNEL(MODE_CHANNEL),
        .SWITCH_THRESHOLD(SWITCH_THRESHOLD)
    ) radio_receiver (
        .clk(clk),
        .rst(rst),
        .ibus_rx(ibus_rx),
        .throttle(throttle),
        .roll_command(roll_command),
        .pitch_command(pitch_command),
        .yaw_command(yaw_command),
        .arm_request(arm_request),
        .avoidance_enable(avoidance_enable),
        .frame_valid(frame_valid),
        .rc_valid(rc_valid),
        .channels(channels)
    );

    LidarArray #(
        .CLK_FREQ(CLK_FREQ), .STARTUP_MS(LIDAR_STARTUP_MS),
        .SETTLE_MS(LIDAR_SETTLE_MS), .POLL_MS(LIDAR_POLL_MS),
        .RESPONSE_TIMEOUT_MS(LIDAR_RESPONSE_TIMEOUT_MS),
        .BYTE_TIMEOUT_MS(LIDAR_BYTE_TIMEOUT_MS)
    ) lidar_array (
        .clk(clk), .rst(rst), .lidar_rx(lidar_rx), .lidar_tx(lidar_tx),
        .read_address(lidar_read_address), .read_distance_mm(lidar_distance_mm),
        .image_valid(lidar_image_valid), .frame_valid(lidar_frame_valid),
        .frame_count(lidar_frame_count), .initialized(lidar_initialized),
        .error(lidar_error)
    );

    MotorControl #(
        .CLK_FREQ(CLK_FREQ), .DSHOT_RATE(MOTOR_DSHOT_RATE),
        .UPDATE_HZ(MOTOR_UPDATE_HZ), .STARTUP_MS(MOTOR_STARTUP_MS)
    ) motor_control (
        .clk(clk), .rst(rst), .motors_armed(motors_armed),
        .command_valid(rc_valid && motor_command_valid),
        .front_right_throttle(front_right_throttle), .front_left_throttle(front_left_throttle),
        .back_right_throttle(back_right_throttle), .back_left_throttle(back_left_throttle),
        .esc_front_right(esc_front_right), .esc_front_left(esc_front_left),
        .esc_back_right(esc_back_right), .esc_back_left(esc_back_left),
        .ready(motor_ready), .busy(motor_busy)
    );
endmodule
