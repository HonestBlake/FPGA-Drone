`timescale 1ns / 1ps

// Physical radio + IMU estimator + flight controller + selected ESC outputs.
// Shared design files under Drone/ are used directly; lidar is acquired for USB viewing and display-only NPU inference.
module BatteryLightsFlightTest #(
    parameter integer CLK_FREQ = 100_000_000,
    parameter integer CORE_CLK_DIV = 20,
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
    parameter integer BATTERY_FILTER_SAMPLES = 64,
    parameter integer BATTERY_UPDATE_HZ = 4,
    parameter integer MAX_ROLL_CD = 2000,
    parameter integer MAX_PITCH_CD = 2000,
    parameter integer ATTITUDE_KP = 32,
    parameter integer RATE_KD = 4,
    parameter integer YAW_KP = 8,
    parameter integer YAW_HEADING_HOLD = 1,
    parameter integer ENABLE_HOVER = 1,
    parameter integer ENABLE_POSITION_DIAGNOSTICS = 0,
    parameter integer HOVER_HANDOVER_MS = 500,
    // Confirmed trained labels: 0 Continue, 1 Left, 2 Right, 3 Up, 4 Stop.
    parameter [14:0] NPU_CLASS_ACTIONS = {3'd1,3'd2,3'd4,3'd3,3'd0},
    parameter integer LIDAR_STARTUP_MS = 1000,
    parameter integer LIDAR_SETTLE_MS = 5000,
    parameter integer LIDAR_POLL_MS = 100,
    parameter integer LIDAR_RESPONSE_TIMEOUT_MS = 8000,
    parameter integer LIDAR_BYTE_TIMEOUT_MS = 500,
    parameter integer LIDAR_FRESH_MS = 2000,
    parameter integer IMU_MOUNT_FRD = 0, // Measured level raw Z is approximately +16384.
    // Bench tuning: +/-30deg heading error contributes up to +/-187 counts.
    // Shared controller default remains 8; rate damping stays at YAW_KP=8.
    parameter integer YAW_HEADING_KP = 16,
    parameter integer NPU_MIN_SCORE = 4096,
    parameter integer NPU_MIN_MARGIN = 4096,
    parameter integer NPU_RELATIVE_MARGIN_SHIFT = 2
) (
    input wire clk, btnC, tx,
    input wire lidar_rx, ultrasonic_echo,
    output wire ultrasonic_trigger,
    output wire lidar_tx, usb_uart_tx,
    input wire [15:0] sw,
    input wire battery_vin_p, battery_vin_n,
    output wire [2:0] light_out,
    input wire imu_spi_miso,
    output wire imu_spi_cs_n, imu_spi_sclk, imu_spi_mosi,
    output wire [3:0] esc,
    output wire [6:0] seg,
    output wire dp,
    output wire [3:0] an,
    output wire [15:0] led
);
    // The unpipelined estimator has wide arithmetic. Run the shared control
    // path at 5 MHz from the board's 100 MHz clock, with unchanged nominal real-time
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
            arm_meta <= sw[15]; arm_sync <= arm_meta;
        end
    end

    (* ASYNC_REG = "TRUE" *) reg battery_view_meta, battery_view, error_view_meta, error_view;
    (* ASYNC_REG = "TRUE" *) reg range_view_meta, range_view, lidar_view_meta, lidar_view;
    (* ASYNC_REG = "TRUE" *) reg yaw_view_meta, yaw_view, imu_view_meta, imu_view;
    (* ASYNC_REG = "TRUE" *) reg [2:0] yaw_diag_meta,yaw_diag_view;
    (* ASYNC_REG = "TRUE" *) reg posx_meta,posx_view,posy_meta,posy_view;
    always @(posedge logic_clk) begin
        if (rst) begin
            yaw_diag_meta<=0;yaw_diag_view<=0;
            posx_meta<=0;posx_view<=0;posy_meta<=0;posy_view<=0;
            imu_view_meta <= 0; imu_view <= 0;
            yaw_view_meta <= 0; yaw_view <= 0;
            lidar_view_meta <= 0; lidar_view <= 0;
            range_view_meta <= 0; range_view <= 0;
            battery_view_meta <= 0; battery_view <= 0;
            error_view_meta <= 0; error_view <= 0;
        end
        else begin
            yaw_diag_meta<=sw[6:4];yaw_diag_view<=yaw_diag_meta;
            posx_meta<=sw[11];posx_view<=posx_meta;posy_meta<=sw[12];posy_view<=posy_meta;
            imu_view_meta <= sw[8]; imu_view <= imu_view_meta;
            yaw_view_meta <= sw[7]; yaw_view <= yaw_view_meta;
            lidar_view_meta <= sw[9]; lidar_view <= lidar_view_meta;
            range_view_meta <= sw[10]; range_view <= range_view_meta;
            battery_view_meta <= sw[13]; battery_view <= battery_view_meta;
            error_view_meta <= sw[14]; error_view <= error_view_meta;
        end
    end
    reg heartbeat;
    wire [223:0] rc_channels;
    wire rc_valid, frame_valid;
    wire [15:0] aux_a = rc_channels[6*16 +: 16]; // CH7 SwA.
    wire [15:0] aux_b = rc_channels[7*16 +: 16]; // CH8 SwB.
    wire [15:0] aux_c = rc_channels[8*16 +: 16]; // CH9 SwC.
    wire [15:0] aux_d = rc_channels[9*16 +: 16]; // CH10 SwD.
    function aux_valid;
        input [15:0] value;
        begin aux_valid = value >= 900 && value <= 2100; end
    endfunction
    // AI stays display-only; CH10 SwD controls the ultrasonic height hold.
    wire ai_override_request = !rst && rc_valid && aux_valid(aux_a) && aux_a >= 1750;
    wire hover_request = !rst && rc_valid && aux_valid(aux_d) && aux_d >= 1750;
    wire [2:0] light_sync = {
        !rst && rc_valid && aux_valid(aux_b) && aux_b >= 1750,
        !rst && rc_valid && aux_valid(aux_c) && aux_c >= 1750,
        !rst && rc_valid && aux_valid(aux_c) && aux_c >= 1250
    };
    // light_out[0]=headlights, [1]=landing lights, [2]=prop lights.
    assign light_out = rst ? 3'b0 : {light_sync[2] && heartbeat, light_sync[1:0]};
    wire [6:0] battery_percent;
    wire [9:0] battery_percent_tenths;
    wire [15:0] battery_mv;
    wire battery_valid;
    // 1:16 divider; voltage-based usable-range estimate, not coulomb counting.
    BatteryMonitor #(.CLK_FREQ(LOGIC_FREQ), .XADC_CHANNEL(6),
        .EMPTY_MV(10500), .FULL_MV(12600),
        .FILTER_SAMPLES(BATTERY_FILTER_SAMPLES), .UPDATE_HZ(BATTERY_UPDATE_HZ)) battery (
        .clk(logic_clk), .rst(rst), .battery_vin_p(battery_vin_p),
        .battery_vin_n(battery_vin_n), .battery_raw(), .battery_mv(battery_mv),
        .battery_percent(battery_percent), .battery_percent_tenths(battery_percent_tenths), .battery_valid(battery_valid),
        .battery_sample_valid()
    );
    wire [5:0] lidar_read_address;
    wire [15:0] lidar_distance_mm;
    wire [31:0] lidar_frame_count;
    wire lidar_image_valid, lidar_frame_valid, lidar_initialized, lidar_error;
    LidarArray #(.CLK_FREQ(LOGIC_FREQ), .STARTUP_MS(LIDAR_STARTUP_MS),
        .SETTLE_MS(LIDAR_SETTLE_MS), .POLL_MS(LIDAR_POLL_MS),
        .RESPONSE_TIMEOUT_MS(LIDAR_RESPONSE_TIMEOUT_MS), .BYTE_TIMEOUT_MS(LIDAR_BYTE_TIMEOUT_MS)) lidar (
        .clk(logic_clk), .rst(rst), .lidar_rx(lidar_rx), .lidar_tx(lidar_tx),
        .read_address(lidar_read_address), .read_distance_mm(lidar_distance_mm),
        .image_valid(lidar_image_valid), .frame_valid(lidar_frame_valid),
        .frame_count(lidar_frame_count), .initialized(lidar_initialized), .error(lidar_error));
    localparam integer LIDAR_FRESH_CYCLES=(LOGIC_FREQ/1000)*LIDAR_FRESH_MS;
    reg [31:0] lidar_age;
    reg lidar_failed;
    always @(posedge logic_clk) begin
        if(rst) begin lidar_age<=LIDAR_FRESH_CYCLES; lidar_failed<=0; end
        else begin
            if(lidar_frame_valid) begin lidar_age<=0; lidar_failed<=0; end
            else if(lidar_age<LIDAR_FRESH_CYCLES) lidar_age<=lidar_age+1'b1;
            if(lidar_error) lidar_failed<=1;
        end
    end
    wire lidar_healthy=lidar_initialized && lidar_image_valid && !lidar_failed &&
                       lidar_age<LIDAR_FRESH_CYCLES;
    wire npu_pixel_valid, npu_available, npu_action_valid;
    wire [5:0] npu_pixel_address;
    wire [15:0] npu_pixel_mm;
    wire [2:0] npu_action;
    wire [2:0] npu_command = npu_action<=4 ? NPU_CLASS_ACTIONS[npu_action*3 +: 3] : 3'd7;
    NPU #(.ENABLE(1),.MIN_SCORE(NPU_MIN_SCORE),.MIN_MARGIN(NPU_MIN_MARGIN),
          .RELATIVE_MARGIN_SHIFT(NPU_RELATIVE_MARGIN_SHIFT)) npu (
        .clk(logic_clk), .rst(rst), .healthy(lidar_healthy),
        .pixel_valid(npu_pixel_valid), .pixel_address(npu_pixel_address), .pixel_mm(npu_pixel_mm),
        .available(npu_available), .action_valid(npu_action_valid), .action(npu_action));
    wire [15:0] lidar_center_mm;
    wire lidar_center_valid;
    DepthFrameStream #(.CLK_FREQ(LOGIC_FREQ)) depth_stream(
        .clk(logic_clk), .rst(rst), .frame_valid(lidar_frame_valid),
        .frame_count(lidar_frame_count), .read_address(lidar_read_address),
        .distance_mm(lidar_distance_mm), .center_mm(lidar_center_mm),
        .center_valid(lidar_center_valid), .pixel_valid(npu_pixel_valid),
        .pixel_address(npu_pixel_address), .pixel_mm(npu_pixel_mm), .uart_tx(usb_uart_tx));
    wire [15:0] ultrasonic_distance_mm;
    wire ultrasonic_valid, ultrasonic_done, ultrasonic_sample_valid;
    wire [2:0] ultrasonic_fault;
    // Bench scope mode: 20us pulses every 65ms, even if echo is stuck high.
    Ultrasonic #(.CLK_FREQ(LOGIC_FREQ), .TRIGGER_US(20),
        .TRIGGER_ON_STUCK_ECHO(1)) ultrasonic(
        .clk(logic_clk), .rst(rst), .enable(1'b1), .echo(ultrasonic_echo),
        .trigger(ultrasonic_trigger), .busy(), .distance_mm(ultrasonic_distance_mm),
        .sample_valid(ultrasonic_sample_valid), .measurement_done(ultrasonic_done),
        .valid(ultrasonic_valid), .fault_code(ultrasonic_fault));
    // Manual stick remains separate from the selected flight collective.
    wire [9:0] throttle;
    wire throttle_idle = throttle < 10'd10;
    wire signed [10:0] roll_command, pitch_command, yaw_command;
    wire arm_request = arm_sync;
    RadioReceiver #(.CLK_FREQ(LOGIC_FREQ), .BAUD_RATE(BAUD_RATE),
                    .TIMEOUT_MS(RC_TIMEOUT_MS)) radio (
        .clk(logic_clk), .rst(rst), .ibus_rx(tx), .throttle(throttle),
        .roll_command(roll_command), .pitch_command(pitch_command),
        .yaw_command(yaw_command), .arm_request(),
        .avoidance_enable(), .frame_valid(frame_valid), .rc_valid(rc_valid), .channels(rc_channels)
    );
    wire imu_detected, imu_init_done, imu_calibrated, imu_valid, imu_fault, imu_sample_valid;
    wire imu_bus_fault;
    wire imu_accel_clipped;
    wire [7:0] imu_fault_register, imu_fault_expected, imu_fault_observed;
    wire imu_raw_valid;
    wire [3:0] imu_fault_reason;
    wire signed [15:0] raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
    wire signed [15:0] roll_cd, pitch_cd, roll_rate, pitch_rate, yaw_rate;
    IMU #(.CLK_FREQ(LOGIC_FREQ), .SPI_CLK_DIV(IMU_SPI_CLK_DIV),
        .POWERUP_CYCLES(IMU_POWERUP_CYCLES), .RESET_CYCLES(IMU_RESET_CYCLES),
        .WAKE_CYCLES(IMU_WAKE_CYCLES), .CALIBRATION_SAMPLES(IMU_CALIBRATION_SAMPLES),
        .TIMEOUT_MS(IMU_TIMEOUT_MS), .MOUNT_FRD(IMU_MOUNT_FRD), .RECOVERY_ENABLED(1),
        .ALLOW_ACCEL_CLIPPING(1)) imu (
        .clk(logic_clk), .rst(rst), .spi_miso(imu_spi_miso), .spi_cs_n(imu_spi_cs_n),
        .spi_sclk(imu_spi_sclk), .spi_mosi(imu_spi_mosi), .raw_valid(imu_raw_valid), .busy(),
        .init_done(imu_init_done), .detected(imu_detected),
        .sample_valid(imu_sample_valid), .valid(imu_valid),
        .calibrated(imu_calibrated), .fault(imu_fault),
        .recover_request(rc_valid && !arm_request && throttle_idle), .fault_reason(imu_fault_reason),
        .bus_fault(imu_bus_fault), .accel_clipped(imu_accel_clipped), .fault_register(imu_fault_register),
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
    wire fc_command_valid, fc_armed, fc_can_arm, fc_imu_fresh;
    wire signed [15:0] measured_yaw_rate_cdps,yaw_correction_counts,heading_error_cd;
    wire signed [15:0] measured_pitch_cd,measured_pitch_rate_cdps,pitch_correction_counts;
    wire [10:0] fc_fr, fc_fl, fc_br, fc_bl;
    wire hover_active,hover_pickup_wait,hover_handover,hover_fault;
    wire [9:0] hover_throttle;
    wire [15:0] hover_target_mm;
    wire [9:0] flight_throttle=hover_active ? hover_throttle : throttle;
    wire flight_throttle_idle=flight_throttle<10;
    wire hover_led=ENABLE_HOVER ?
        ((hover_active && !hover_pickup_wait && !hover_handover && !hover_fault) ||
         ((hover_active || hover_request || hover_fault) && heartbeat)) : hover_request;
    generate if(ENABLE_HOVER)begin : g_hover
        HoverController #(.CLK_FREQ(LOGIC_FREQ),.CONTROL_HZ(CONTROL_HZ),.HANDOVER_MS(HOVER_HANDOVER_MS)) hover (
            .clk(logic_clk),.rst(rst),.enable(hover_request),.armed(fc_armed),
            .control_valid(fc_command_valid && sw_sync==4'hf),
            .attitude_ok(imu_valid && roll_cd>=-1500 && roll_cd<=1500 && pitch_cd>=-1500 && pitch_cd<=1500),
            .sample_valid(ultrasonic_sample_valid),.range_valid(ultrasonic_valid),
            .distance_mm(ultrasonic_distance_mm),.manual_throttle(throttle),.control_tick(control_tick),
            .active(hover_active),.range_fresh(),.throttle_out(hover_throttle),
            .target_height_mm(hover_target_mm),.vertical_speed_mmps(),.fault(hover_fault),
            .pickup_wait(hover_pickup_wait),.handover_active(hover_handover));
    end else begin : g_no_hover
        assign hover_active=0;assign hover_pickup_wait=0;assign hover_handover=0;assign hover_fault=0;
        assign hover_throttle=throttle;assign hover_target_mm=0;
    end endgenerate
    wire signed [15:0] roll_setpoint, pitch_setpoint, yaw_setpoint;
    FlightController #(.CLK_FREQ(LOGIC_FREQ), .FULL_MANUAL_CONTROL(1), .YAW_MIX_SIGN(-1),
        .IMU_TIMEOUT_MS(IMU_TIMEOUT_MS), .MAX_ROLL_CD(MAX_ROLL_CD), .MAX_PITCH_CD(MAX_PITCH_CD),
        .ATTITUDE_KP(ATTITUDE_KP), .RATE_KD(RATE_KD), .YAW_KP(YAW_KP),
        .YAW_HEADING_HOLD(YAW_HEADING_HOLD), .YAW_HEADING_KP(YAW_HEADING_KP)) controller (
        .clk(logic_clk), .rst(rst), .control_tick(control_tick),
        .throttle(flight_throttle_idle ? 10'd0 : flight_throttle),
        .roll_command(roll_command), .pitch_command(pitch_command), .yaw_command(yaw_command),
        .arm_request(arm_request), .avoidance_enable(1'b0), .frame_valid(frame_valid),
        .rc_valid(rc_valid && selection_ready), .motor_ready(motor_ready),
        .imu_sample_valid(imu_sample_valid), .imu_valid(imu_valid),
        .imu_calibrated(imu_calibrated), .imu_fault(imu_fault),
        .imu_roll_cd(roll_cd), .imu_pitch_cd(pitch_cd),
        .imu_roll_rate_cdps(roll_rate), .imu_pitch_rate_cdps(pitch_rate), .imu_yaw_rate_cdps(yaw_rate),
        .npu_available(1'b0), .npu_action_valid(1'b0), .npu_action(3'd0),
        .available(), .command_valid(fc_command_valid), .imu_fresh(fc_imu_fresh), .npu_fresh(),
        .motors_armed(fc_armed), .can_arm(fc_can_arm), .avoidance_active(), .applied_action(),
        .roll_setpoint_cd(roll_setpoint), .pitch_setpoint_cd(pitch_setpoint),
        .measured_yaw_rate_cdps(measured_yaw_rate_cdps),
        .yaw_correction_counts(yaw_correction_counts),.heading_error_cd(heading_error_cd),
        .measured_pitch_cd(measured_pitch_cd),.measured_pitch_rate_cdps(measured_pitch_rate_cdps),
        .pitch_correction_counts(pitch_correction_counts),
        .yaw_rate_setpoint_cdps(yaw_setpoint), .thrust_setpoint(),
        .front_right_throttle(fc_fr), .front_left_throttle(fc_fl),
        .back_right_throttle(fc_br), .back_left_throttle(fc_bl)
    );

    MotorControl #(.CLK_FREQ(LOGIC_FREQ), .UPDATE_HZ(MOTOR_UPDATE_HZ),
                   .STARTUP_MS(MOTOR_STARTUP_MS)) motors (
        .clk(logic_clk), .rst(rst), .motors_armed(fc_armed && !flight_throttle_idle && selection_ready),
        .command_valid(fc_command_valid && rc_valid && imu_valid),
        .front_right_throttle(sw_sync[3] ? fc_fr : 11'd0),
        .front_left_throttle(sw_sync[1] ? fc_fl : 11'd0),
        .back_right_throttle(sw_sync[2] ? fc_br : 11'd0),
        .back_left_throttle(sw_sync[0] ? fc_bl : 11'd0),
        // Confirmed physical order: SW0 BL, SW1 FL, SW2 BR, SW3 FR.
        // JB1..4 are numbered channels 0..3.
        .esc_front_right(esc[3]), .esc_front_left(esc[1]),
        .esc_back_right(esc[2]), .esc_back_left(esc[0]),
        .ready(motor_ready), .busy()
    );

    wire signed [31:0] position_x_mm,position_y_mm;
    wire position_calibrated,position_degraded,position_valid;
    generate if(ENABLE_POSITION_DIAGNOSTICS)begin : g_position
    PositionEstimator #(.CLK_FREQ(LOGIC_FREQ), .TRACK_Z(0)) position_diagnostic(
        .clk(logic_clk),.rst(rst),.enable(fc_armed && !flight_throttle_idle),
        .sample_valid(imu_sample_valid),.imu_valid(imu_valid),.accel_clipped(imu_accel_clipped),
        .accel_x(raw_ax),.accel_y(IMU_MOUNT_FRD ? -raw_ay : raw_ay),
        .accel_z(IMU_MOUNT_FRD ? -raw_az : raw_az),.roll_cd(roll_cd),.pitch_cd(pitch_cd),
        .yaw_rate_cdps(yaw_rate),.calibrated(position_calibrated),.degraded(position_degraded),
        .estimate_valid(position_valid),.x_mm(position_x_mm),.y_mm(position_y_mm),.z_mm(),
        .vx_mmps(),.vy_mmps(),.vz_mmps());
    end else begin : g_no_position
        assign position_x_mm=0;assign position_y_mm=0;assign position_valid=0;
        assign position_calibrated=0;assign position_degraded=0;
    end endgenerate
    // Diagnostic-only local centimetres, signed and limited to three digits.
    wire position_view=ENABLE_POSITION_DIAGNOSTICS && (posx_view || posy_view);
    wire signed [31:0] position_value_mm=posx_view ? position_x_mm : position_y_mm;
    wire [31:0] position_magnitude_mm=position_value_mm[31] ? -position_value_mm : position_value_mm;
    wire [15:0] position_cm=position_magnitude_mm/10>999 ? 16'd999 : position_magnitude_mm/10;
    // SW4/SW5/SW6 retain yaw speed/correction/heading error. Adding SW8 selects
    // pitch angle/correction/speed instead (+nose-up). SW8 alone remains roll.
    wire yaw_diagnostics_view=|yaw_diag_view;
    wire diag_first_selected=yaw_diag_view[0];
    wire diag_correction_selected=!diag_first_selected && yaw_diag_view[1];
    wire diag_pitch_selected=imu_view;
    wire diag_rate_selected=diag_pitch_selected ? (!diag_first_selected && !diag_correction_selected) : diag_first_selected;
    wire signed [15:0] yaw_diag_raw=diag_pitch_selected ?
        (diag_first_selected ? measured_pitch_cd : diag_correction_selected ? pitch_correction_counts : measured_pitch_rate_cdps) :
        (diag_first_selected ? measured_yaw_rate_cdps : diag_correction_selected ? yaw_correction_counts : heading_error_cd);
    wire [15:0] yaw_diag_abs=yaw_diag_raw[15] ? -yaw_diag_raw : yaw_diag_raw;
    wire [16:0] yaw_diag_tenths=({1'b0,yaw_diag_abs}+17'd5)/10;
    wire [16:0] yaw_diag_whole=({1'b0,yaw_diag_abs}+17'd50)/100;
    wire yaw_diag_decimal=diag_rate_selected ? yaw_diag_tenths<=999 : !diag_correction_selected;
    wire [16:0] yaw_diag_number=diag_rate_selected ?
        (yaw_diag_decimal ? yaw_diag_tenths : yaw_diag_whole) :
        diag_correction_selected ? {1'b0,yaw_diag_abs} : yaw_diag_tenths;
    wire diag_measured_selected=diag_rate_selected || (diag_pitch_selected && diag_first_selected);
    wire yaw_diag_valid=(diag_measured_selected ? fc_imu_fresh : fc_command_valid) && yaw_diag_number<=999;
    // SW11/SW12 are free when the optional X/Y estimator is excluded.
    wire rc_roll_view=!ENABLE_POSITION_DIAGNOSTICS && posx_view;
    wire rc_pitch_view=!ENABLE_POSITION_DIAGNOSTICS && posy_view;
    wire rc_high_view=rc_roll_view || rc_pitch_view;
    wire signed [10:0] rc_axis_command=rc_roll_view ? roll_command : rc_pitch_view ? pitch_command : yaw_command;
    wire [10:0] rc_axis_magnitude=rc_axis_command[10] ? -rc_axis_command : rc_axis_command;
    wire [15:0] roll_magnitude_cd = roll_cd[15] ? (~roll_cd + 16'd1) : roll_cd;
    wire [16:0] roll_degrees = ({1'b0,roll_magnitude_cd}+17'd50)/100;
    wire roll_view_active = imu_view && !error_view && !configuring &&
                            !yaw_diagnostics_view && !position_view && !rc_high_view && !range_view && !lidar_view && imu_valid;
    wire [15:0] display_number = yaw_diagnostics_view ? yaw_diag_number[15:0] : position_view ? position_cm : rc_high_view ? {5'd0,rc_axis_magnitude} : range_view ? ultrasonic_distance_mm :
        lidar_view ? lidar_center_mm :
        imu_view ? roll_degrees[15:0] :
        battery_view ? {6'b0,battery_percent_tenths} : {5'b0,rc_axis_magnitude};
    reg [31:0] bcd;
    integer bit_index, digit_index;
    always @* begin
        bcd = {16'b0, display_number};
        for (bit_index = 0; bit_index < 16; bit_index = bit_index + 1) begin
            for (digit_index = 0; digit_index < 4; digit_index = digit_index + 1)
                if (bcd[16 + digit_index*4 +: 4] >= 5)
                    bcd[16 + digit_index*4 +: 4] = bcd[16 + digit_index*4 +: 4] + 4'd3;
            bcd = bcd << 1;
        end
    end
    // On a bus fault, show register.observed in hex (e.g. 00.FF).
    // Estimator faults show E0.01 timeout, E0.02 gyro clip, E0.03 accel clip,
    // E0.04 tilt, E0.05 rate overflow, E0.06 bus fault, E0.07 sensor not ready.
    // Readiness reflects all live arm prerequisites, including idle throttle.
    // The controller still requires its explicit OFF-to-ON arming sequence.
    wire ready_to_arm = imu_valid && imu_calibrated && motor_ready &&
                        rc_valid && selection_ready && throttle_idle;
    wire configuring = (!imu_init_done || !imu_calibrated) && !imu_fault;
    wire error_present = imu_fault || (!configuring && (!rc_valid || !battery_valid)) || ultrasonic_fault != 0 || hover_fault;
    wire [15:0] error_code = imu_fault ?
        (imu_bus_fault ? {imu_fault_register,imu_fault_observed} : {12'he00,imu_fault_reason}) :
        !configuring && !rc_valid ? 16'he010 :
        !configuring && !battery_valid ? 16'he011 :
        ultrasonic_fault != 0 ? 16'he030 : hover_fault ? 16'he031 : 16'he000;
    wire [15:0] display_value = configuring ? 16'hc0f6 :
        error_view ? error_code :
        yaw_diagnostics_view ? (yaw_diag_valid ? {yaw_diag_raw[15] ? 4'hd : 4'hc,bcd[27:16]} : 16'hffff) :
        position_view ? (position_valid ? bcd[31:16] : 16'hffff) :
        rc_high_view ? (rc_valid ? {rc_axis_command[10] ? 4'hd : 4'hc,bcd[27:16]} : 16'hffff) :
        range_view ? (ultrasonic_valid ? bcd[31:16] : 16'hffff) :
        lidar_view ? (npu_available && npu_command<=4 ? (16'hb100 + {13'd0,npu_command}) : 16'hffff) :
        imu_view ? (imu_valid ? bcd[31:16] : 16'hffff) :
        battery_view ? (battery_valid ? bcd[31:16] : 16'hffff) :
        yaw_view ? (rc_valid ? {rc_axis_command[10] ? 4'hd : 4'hc,bcd[27:16]} : 16'hffff) :
        fc_armed ? 16'ha17d : fc_can_arm ? 16'hadbf : 16'hffff;
    wire battery_view_active = !configuring && !error_view && !range_view &&
        !yaw_diagnostics_view && !position_view && !rc_high_view && !lidar_view && !imu_view && battery_view && battery_valid;
    wire [6:0] display_seg;
    SevenSegmentDisplay #(.CLK_FREQ(LOGIC_FREQ), .REFRESH_HZ(DISPLAY_REFRESH_HZ),
        .STATUS_WORDS(1), .RC_AXIS_WORDS(1)) display (
        .clk(logic_clk), .rst(rst), .write(1'b1), .value(display_value),
        .digit_enable(4'b1111),
        .decimal_points(!configuring && error_view && imu_fault ? 4'b0100 :
                        (!configuring && !error_view && yaw_diagnostics_view && yaw_diag_valid && yaw_diag_decimal) ? 4'b0010 :
                        battery_view_active ? 4'b0010 : 4'b0000),
        .seg(display_seg), .dp(dp), .an(an)
    );
    wire position_view_active=position_view && position_valid && !yaw_diagnostics_view && !rst && !configuring && !error_view;
    assign seg = position_view_active && an==4'b0111 ? (position_value_mm[31] ? 7'h3f : 7'h7f) :
        roll_view_active && !rst && an == 4'b0111 ?
        (roll_cd[15] ? 7'h3f : 7'h7f) :
        (display_value == 16'hffff && !rst && an != 4'hf) ? 7'h3f : display_seg;
    localparam integer HEARTBEAT_CYCLES = LOGIC_FREQ/2;
    localparam integer HEARTBEAT_WIDTH = (HEARTBEAT_CYCLES > 1) ? $clog2(HEARTBEAT_CYCLES) : 1;
    reg [HEARTBEAT_WIDTH-1:0] heartbeat_count;
    always @(posedge logic_clk) begin
        if (rst) begin heartbeat_count <= 0; heartbeat <= 0; end
        else if (heartbeat_count == HEARTBEAT_CYCLES-1) begin
            heartbeat_count <= 0; heartbeat <= !heartbeat;
        end else heartbeat_count <= heartbeat_count+1'b1;
    end
    // Minimal operator indicators; LD15 reflects actual controller arming.
    wire imu_healthy = imu_valid && imu_calibrated && !imu_fault;
    wire battery_low = battery_valid && battery_percent <= 7'd20;
    assign led = rst ? 16'b0 : {(fc_armed || (fc_can_arm && heartbeat)),
        (error_view || (error_present && heartbeat)),
        (battery_valid && (!battery_low || heartbeat)),
        hover_led, ai_override_request, (ultrasonic_valid || heartbeat), (lidar_healthy || heartbeat),
        (imu_healthy ? 1'b1 : heartbeat),
        (rc_valid ? 1'b1 : heartbeat), light_sync[0], light_sync[1], light_out[2], (sw_sync | {4{heartbeat}})};
endmodule
