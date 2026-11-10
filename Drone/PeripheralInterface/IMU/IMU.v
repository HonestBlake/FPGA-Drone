`timescale 1ns / 1ps
module IMU #(
    parameter integer CLK_FREQ = 100_000_000,
    parameter integer SPI_CLK_DIV = (CLK_FREQ + 9_999_999) / 10_000_000,
    parameter integer POWERUP_CYCLES = (CLK_FREQ/1000)*100,
    parameter integer RESET_CYCLES = (CLK_FREQ/1000)*100,
    parameter integer WAKE_CYCLES = (CLK_FREQ/1000)*35,
    parameter integer POLL_HZ = 500,
    parameter integer CALIBRATION_SAMPLES = 256,
    parameter integer TIMEOUT_MS = 100,
    parameter integer MOUNT_FRD = 0, // Sensor X forward, Y right, Z down when 1.
    parameter integer RECOVERY_ENABLED = 0
) (
    input wire clk, rst, spi_miso,
    output wire spi_cs_n, spi_sclk, spi_mosi,
    output wire raw_valid, busy, init_done, detected, sample_valid, valid, calibrated, fault,
    output wire signed [15:0] accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z,
    output wire signed [15:0] roll_cd, pitch_cd, roll_rate_cdps, pitch_rate_cdps, yaw_rate_cdps,
    output wire bus_fault,
    output wire [7:0] fault_register, fault_expected, fault_observed,
    input wire recover_request,
    output wire [3:0] fault_reason
);
    localparam integer POLL_CYCLES = CLK_FREQ/POLL_HZ;
    reg [31:0] poll_count;
    reg start;
    wire spi_error;
    assign bus_fault = spi_error;
    always @(posedge clk) begin
        if (rst) begin poll_count <= 0; start <= 0; end
        else begin
            start <= 0;
            if (!init_done) poll_count <= 0;
            else if (poll_count >= POLL_CYCLES-1) begin
                poll_count <= 0;
                if (!busy) start <= 1;
            end else poll_count <= poll_count+1'b1;
        end
    end
    imu_spi #(.CLK_FREQ(CLK_FREQ), .CLK_DIV(SPI_CLK_DIV),
        .POWERUP_CYCLES(POWERUP_CYCLES), .RESET_CYCLES(RESET_CYCLES), .WAKE_CYCLES(WAKE_CYCLES)) driver (
        .clk(clk), .reset(rst), .start(start), .spi_cs_n(spi_cs_n),
        .spi_sclk(spi_sclk), .spi_mosi(spi_mosi), .spi_miso(spi_miso),
        .accel_x(accel_x), .accel_y(accel_y), .accel_z(accel_z),
        .gyro_x(gyro_x), .gyro_y(gyro_y), .gyro_z(gyro_z),
        .data_valid(raw_valid), .busy(busy), .init_done(init_done), .imu_detected(detected), .error(spi_error),
        .fault_register(fault_register), .fault_expected(fault_expected), .fault_observed(fault_observed)
    );
    // Estimator expects X forward, Y left, Z up. Rotate both acceleration and
    // angular velocity by 180 degrees about X for the actual FRD mounting.
    // Raw telemetry above remains in the physical sensor's coordinate system.
    wire signed [15:0] estimate_ay = MOUNT_FRD ? -accel_y : accel_y;
    wire signed [15:0] estimate_az = MOUNT_FRD ? -accel_z : accel_z;
    wire signed [15:0] estimate_gy = MOUNT_FRD ? -gyro_y : gyro_y;
    wire signed [15:0] estimate_gz = MOUNT_FRD ? -gyro_z : gyro_z;
    ImuEstimator #(.CLK_FREQ(CLK_FREQ), .CALIBRATION_SAMPLES(CALIBRATION_SAMPLES),
        .TIMEOUT_MS(TIMEOUT_MS), .RECOVERY_ENABLED(RECOVERY_ENABLED)) estimator (
        .clk(clk), .rst(rst), .raw_valid(raw_valid), .sensor_ready(init_done && detected),
        .sensor_fault(spi_error), .accel_x(accel_x), .accel_y(estimate_ay), .accel_z(estimate_az),
        .gyro_x(gyro_x), .gyro_y(estimate_gy), .gyro_z(estimate_gz), .sample_valid(sample_valid),
        .valid(valid), .calibrated(calibrated), .fault(fault), .roll_cd(roll_cd), .pitch_cd(pitch_cd),
        .recover_request(recover_request), .fault_reason(fault_reason),
        .roll_rate_cdps(roll_rate_cdps), .pitch_rate_cdps(pitch_rate_cdps), .yaw_rate_cdps(yaw_rate_cdps)
    );
    // synthesis translate_off
    initial begin
        if ((MOUNT_FRD != 0 && MOUNT_FRD != 1) || POLL_HZ < 1 || POLL_HZ > 562 ||
            POLL_CYCLES < 272*SPI_CLK_DIV+200 || POLL_CYCLES >= (CLK_FREQ/1000)*TIMEOUT_MS)
            $fatal(1, "IMU: poll rate must fit SPI transfers and freshness timeout");
    end
    // synthesis translate_on
endmodule
