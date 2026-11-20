`timescale 1ns / 1ps

// Physical radio + IMU estimator + flight controller + selected ESC outputs.
// Shared design files under Drone/ are used directly; no lidar or NPU instance.
module FlightTest #(
    parameter integer CLK_FREQ = 100_000_000,
    parameter integer CORE_CLK_DIV = 10,
    parameter integer BAUD_RATE = 115_200,
    parameter integer RC_TIMEOUT_MS = 100,
    parameter integer CONTROL_HZ = 1_000,
    // Use 1 MHz for jumper-wire/level-shifter margin during hardware diagnosis.
    parameter integer IMU_SPI_CLK_DIV = ((CLK_FREQ / CORE_CLK_DIV) + 1_999_999) / 2_000_000,
    parameter integer IMU_POWERUP_CYCLES = (CLK_FREQ/CORE_CLK_DIV/1000)*100,
    parameter integer IMU_RESET_CYCLES = (CLK_FREQ/CORE_CLK_DIV/1000)*100,
    parameter integer IMU_WAKE_CYCLES = (CLK_FREQ/CORE_CLK_DIV/1000)*35,
    parameter integer IMU_CALIBRATION_SAMPLES = 256,
    parameter integer IMU_TIMEOUT_MS = 100,
    parameter integer MOTOR_UPDATE_HZ = 1_000,
    parameter integer MOTOR_STARTUP_MS = 1_000,
    parameter integer DISPLAY_REFRESH_HZ = 250,
    parameter integer MAX_ROLL_CD = 2000,
    parameter integer MAX_PITCH_CD = 2000,
    parameter integer ATTITUDE_KP = 32,
    parameter integer RATE_KD = 4,
    parameter integer YAW_KP = 8,
    parameter integer IMU_MOUNT_FRD = 0 // Measured level raw Z is approximately +16384.
) (
    input wire clk, btnC, tx,
    input wire [4:0] sw,
    input wire imu_spi_miso,
    output wire imu_spi_cs_n, imu_spi_sclk, imu_spi_mosi,
    output wire [3:0] esc,
    output wire [6:0] seg,
    output wire dp,
    output wire [3:0] an,
    output wire [15:0] led
);
    // The unpipelined estimator has wide arithmetic. Run the shared control
    // path at 10 MHz from the board's 100 MHz clock, with unchanged real-time
    // polling/control/UART/DShot rates. XDC declares this generated clock.
    localparam integer LOGIC_FREQ = (CORE_CLK_DIV > 0) ? CLK_FREQ/CORE_CLK_DIV : CLK_FREQ;
    wire logic_clk;
    generate
        if (CORE_CLK_DIV == 1) begin : g_direct_clock
            assign logic_clk = clk;
        end else begin : g_core_clock
            localparam integer HALF_CYCLES = CORE_CLK_DIV/2;
            localparam integer DIV_WIDTH = (HALF_CYCLES > 1) ? $clog2(HALF_CYCLES) : 1;
            reg [DIV_WIDTH-1:0] core_count = 0;
            reg core_phase = 0;
            always @(posedge clk) begin
                if (core_count == HALF_CYCLES-1) begin
                    core_count <= 0;
                    core_phase <= !core_phase;
                end else core_count <= core_count+1'b1;
            end
            assign logic_clk = core_phase;
        end
    endgenerate
    // synthesis translate_off
    initial begin
        if (CORE_CLK_DIV < 1 || (CORE_CLK_DIV != 1 && CORE_CLK_DIV % 2 != 0) ||
            CLK_FREQ % CORE_CLK_DIV != 0)
            $fatal(1,"FlightTest: require an even clock divider (or bypass 1)");
    end
    // synthesis translate_on
    reg [7:0] startup_reset = 8'hff;
    (* ASYNC_REG = "TRUE" *) reg reset_meta = 1'b1, reset_sync = 1'b1;
    (* ASYNC_REG = "TRUE" *) reg [3:0] sw_meta, sw_sync;
    (* ASYNC_REG = "TRUE" *) reg arm_meta, arm_sync;
    wire rst = (|startup_reset) || reset_sync;
    always @(posedge logic_clk) begin
        startup_reset <= {startup_reset[6:0], 1'b0};
        reset_meta <= btnC;
        reset_sync <= reset_meta;
        if (rst) begin
            sw_meta <= 0; sw_sync <= 0; arm_meta <= 0; arm_sync <= 0;
        end else begin
            sw_meta <= sw[3:0]; sw_sync <= sw_meta;
            arm_meta <= sw[4]; arm_sync <= arm_meta;
        end
    end

    wire [9:0] throttle;
    wire throttle_idle = throttle < 10'd10;
    wire signed [10:0] roll_command, pitch_command, yaw_command;
    wire rc_valid, frame_valid;
    wire arm_request = arm_sync;
    RadioReceiver #(.CLK_FREQ(LOGIC_FREQ), .BAUD_RATE(BAUD_RATE),
                    .TIMEOUT_MS(RC_TIMEOUT_MS)) radio (
        .clk(logic_clk), .rst(rst), .ibus_rx(tx), .throttle(throttle),
        .roll_command(roll_command), .pitch_command(pitch_command),
        .yaw_command(yaw_command), .arm_request(),
        .avoidance_enable(), .frame_valid(frame_valid), .rc_valid(rc_valid), .channels()
    );
    wire imu_detected, imu_init_done, imu_calibrated, imu_valid, imu_fault, imu_sample_valid;
    wire imu_bus_fault;
    wire [7:0] imu_fault_register, imu_fault_expected, imu_fault_observed;
    wire imu_raw_valid;
    wire [3:0] imu_fault_reason;
    wire signed [15:0] raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
    wire signed [15:0] roll_cd, pitch_cd, roll_rate, pitch_rate, yaw_rate;
    IMU #(.CLK_FREQ(LOGIC_FREQ), .SPI_CLK_DIV(IMU_SPI_CLK_DIV),
        .POWERUP_CYCLES(IMU_POWERUP_CYCLES), .RESET_CYCLES(IMU_RESET_CYCLES),
        .WAKE_CYCLES(IMU_WAKE_CYCLES), .CALIBRATION_SAMPLES(IMU_CALIBRATION_SAMPLES),
        .TIMEOUT_MS(IMU_TIMEOUT_MS), .MOUNT_FRD(IMU_MOUNT_FRD), .RECOVERY_ENABLED(1)) imu (
        .clk(logic_clk), .rst(rst), .spi_miso(imu_spi_miso), .spi_cs_n(imu_spi_cs_n),
        .spi_sclk(imu_spi_sclk), .spi_mosi(imu_spi_mosi), .raw_valid(imu_raw_valid), .busy(),
        .init_done(imu_init_done), .detected(imu_detected),
        .sample_valid(imu_sample_valid), .valid(imu_valid),
        .calibrated(imu_calibrated), .fault(imu_fault),
        .recover_request(rc_valid && !arm_request && throttle_idle), .fault_reason(imu_fault_reason),
        .bus_fault(imu_bus_fault), .fault_register(imu_fault_register),
        .fault_expected(imu_fault_expected), .fault_observed(imu_fault_observed),
        .accel_x(raw_ax), .accel_y(raw_ay), .accel_z(raw_az),
        .gyro_x(raw_gx), .gyro_y(raw_gy), .gyro_z(raw_gz),
        .roll_cd(roll_cd), .pitch_cd(pitch_cd), .roll_rate_cdps(roll_rate),
        .pitch_rate_cdps(pitch_rate), .yaw_rate_cdps(yaw_rate)
    );
    localparam integer RAW_TIMEOUT_CYCLES = (LOGIC_FREQ/1000)*IMU_TIMEOUT_MS;
    reg [31:0] raw_age;
    always @(posedge logic_clk) begin
        if (rst) raw_age <= RAW_TIMEOUT_CYCLES;
        else if (imu_raw_valid) raw_age <= 0;
        else if (raw_age < RAW_TIMEOUT_CYCLES) raw_age <= raw_age+1'b1;
    end
    wire raw_fresh = raw_age < RAW_TIMEOUT_CYCLES;
    wire calibration_waiting = imu_init_done && !imu_calibrated && !imu_fault;
    // Match the estimator's acceptance windows after the selected mounting transform.
    wire signed [15:0] calibration_az = IMU_MOUNT_FRD ? -raw_az : raw_az;
    wire accel_window = raw_fresh && raw_ax >= -16'sd1024 && raw_ax <= 16'sd1024 &&
        raw_ay >= -16'sd1024 && raw_ay <= 16'sd1024 &&
        calibration_az >= 16'sd15564 && calibration_az <= 16'sd17204;
    wire gyro_window = raw_fresh && raw_gx >= -16'sd262 && raw_gx <= 16'sd262 &&
        raw_gy >= -16'sd262 && raw_gy <= 16'sd262 &&
        raw_gz >= -16'sd262 && raw_gz <= 16'sd262;
    wire control_tick;
    TickGenerator #(.CLK_FREQ(LOGIC_FREQ), .TICK_FREQ(CONTROL_HZ)) control_timer
        (.clk(logic_clk), .rst(rst), .tick(control_tick));

    wire motor_ready;
    reg [3:0] accepted_selection;
    reg idle_seen;
    always @(posedge logic_clk) begin
        if (rst) begin accepted_selection <= 0; idle_seen <= 0; end
        else if (!rc_valid || !imu_valid || !motor_ready || sw_sync == 0 ||
                     sw_sync != accepted_selection) begin
            accepted_selection <= sw_sync;
            idle_seen <= 0;
        end else if (throttle_idle) idle_seen <= 1;
    end
    wire selection_ready = idle_seen && (sw_sync != 0) && (sw_sync == accepted_selection);
    wire fc_command_valid, fc_armed;
    wire [10:0] fc_fr, fc_fl, fc_br, fc_bl;
    wire signed [15:0] roll_setpoint, pitch_setpoint, yaw_setpoint;
    FlightController #(.CLK_FREQ(LOGIC_FREQ), .FULL_MANUAL_CONTROL(1), .YAW_MIX_SIGN(-1),
        .IMU_TIMEOUT_MS(IMU_TIMEOUT_MS), .MAX_ROLL_CD(MAX_ROLL_CD), .MAX_PITCH_CD(MAX_PITCH_CD),
        .ATTITUDE_KP(ATTITUDE_KP), .RATE_KD(RATE_KD), .YAW_KP(YAW_KP)) controller (
        .clk(logic_clk), .rst(rst), .control_tick(control_tick),
        .throttle(throttle_idle ? 10'd0 : throttle),
        .roll_command(roll_command), .pitch_command(pitch_command), .yaw_command(yaw_command),
        .arm_request(arm_request), .avoidance_enable(1'b0), .frame_valid(frame_valid),
        .rc_valid(rc_valid && selection_ready), .motor_ready(motor_ready),
        .imu_sample_valid(imu_sample_valid), .imu_valid(imu_valid),
        .imu_calibrated(imu_calibrated), .imu_fault(imu_fault),
        .imu_roll_cd(roll_cd), .imu_pitch_cd(pitch_cd),
        .imu_roll_rate_cdps(roll_rate), .imu_pitch_rate_cdps(pitch_rate), .imu_yaw_rate_cdps(yaw_rate),
        .npu_available(1'b0), .npu_action_valid(1'b0), .npu_action(3'd0),
        .available(), .command_valid(fc_command_valid), .imu_fresh(), .npu_fresh(),
        .motors_armed(fc_armed), .avoidance_active(), .applied_action(),
        .roll_setpoint_cd(roll_setpoint), .pitch_setpoint_cd(pitch_setpoint),
        .yaw_rate_setpoint_cdps(yaw_setpoint), .thrust_setpoint(),
        .front_right_throttle(fc_fr), .front_left_throttle(fc_fl),
        .back_right_throttle(fc_br), .back_left_throttle(fc_bl)
    );

    MotorControl #(.CLK_FREQ(LOGIC_FREQ), .UPDATE_HZ(MOTOR_UPDATE_HZ),
                   .STARTUP_MS(MOTOR_STARTUP_MS)) motors (
        .clk(logic_clk), .rst(rst), .motors_armed(fc_armed && !throttle_idle && selection_ready),
        .command_valid(fc_command_valid && rc_valid && imu_valid),
        .front_right_throttle(sw_sync[3] ? fc_fr : 11'd0),
        .front_left_throttle(sw_sync[1] ? fc_fl : 11'd0),
        .back_right_throttle(sw_sync[2] ? fc_br : 11'd0),
        .back_left_throttle(sw_sync[0] ? fc_bl : 11'd0),
        // User's actual wire order: SW0 BL, SW1 FL, SW2 BR, SW3 FR.
        .esc_front_right(esc[3]), .esc_front_left(esc[1]),
        .esc_back_right(esc[2]), .esc_back_left(esc[0]),
        .ready(motor_ready), .busy()
    );

    reg [25:0] bcd;
    integer bit_index, digit_index;
    always @* begin
        bcd = {16'b0, throttle};
        for (bit_index = 0; bit_index < 10; bit_index = bit_index + 1) begin
            for (digit_index = 0; digit_index < 4; digit_index = digit_index + 1)
                if (bcd[10 + digit_index*4 +: 4] >= 5)
                    bcd[10 + digit_index*4 +: 4] = bcd[10 + digit_index*4 +: 4] + 4'd3;
            bcd = bcd << 1;
        end
    end
    // On a bus fault, show register.observed in hex (e.g. 00.FF).
    // Estimator faults show E0.01 timeout, E0.02 gyro clip, E0.03 accel clip,
    // E0.04 tilt, E0.05 rate overflow, E0.06 bus fault, E0.07 sensor not ready.
    wire [15:0] display_value = imu_fault ?
        (imu_bus_fault ? {imu_fault_register,imu_fault_observed} : {12'he00,imu_fault_reason}) :
        calibration_waiting ? (raw_fresh ? raw_az : 16'hc001) :
        (rc_valid ? bcd[25:10] : 16'hffff);
    wire [6:0] display_seg;
    SevenSegmentDisplay #(.CLK_FREQ(LOGIC_FREQ), .REFRESH_HZ(DISPLAY_REFRESH_HZ)) display (
        .clk(logic_clk), .rst(rst), .write(1'b1), .value(display_value),
        .digit_enable(4'b1111),
        .decimal_points(imu_fault ? 4'b0100 : calibration_waiting ? 4'b0001 : 4'b0000),
        .seg(display_seg), .dp(dp), .an(an)
    );
    assign seg = (!imu_fault && !calibration_waiting && !rc_valid && !rst && an != 4'hf) ? 7'h3f : display_seg;
    localparam integer HEARTBEAT_CYCLES = LOGIC_FREQ/2;
    localparam integer HEARTBEAT_WIDTH = (HEARTBEAT_CYCLES > 1) ? $clog2(HEARTBEAT_CYCLES) : 1;
    reg [HEARTBEAT_WIDTH-1:0] heartbeat_count;
    reg heartbeat;
    always @(posedge logic_clk) begin
        if (rst) begin heartbeat_count <= 0; heartbeat <= 0; end
        else if (heartbeat_count == HEARTBEAT_CYCLES-1) begin
            heartbeat_count <= 0; heartbeat <= !heartbeat;
        end else heartbeat_count <= heartbeat_count+1'b1;
    end
    assign led = {calibration_waiting ? gyro_window : arm_request,
                  calibration_waiting ? accel_window : selection_ready,
                  calibration_waiting ? raw_fresh : fc_command_valid, imu_fault,
                  imu_valid, imu_calibrated, imu_init_done, imu_detected,
                  heartbeat, rc_valid, motor_ready, fc_armed, sw_sync};
endmodule

