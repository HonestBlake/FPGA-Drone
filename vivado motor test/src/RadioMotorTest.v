`timescale 1ns / 1ps

// Basys3 bench test: JB1 iBUS, JA7..JA10 ESC signals. No flight computer.
module RadioMotorTest #(
    parameter integer CLK_FREQ = 100_000_000,
    parameter integer BAUD_RATE = 115_200,
    parameter integer TIMEOUT_MS = 100,
    parameter integer DSHOT_RATE = 300_000,
    parameter integer MOTOR_UPDATE_HZ = 1_000,
    parameter integer MOTOR_STARTUP_MS = 1_000,
    parameter integer DISPLAY_REFRESH_HZ = 250
) (
    input wire clk,
    input wire btnC,
    input wire tx,
    input wire [3:0] sw,
    output wire [3:0] esc,
    output wire [6:0] seg,
    output wire dp,
    output wire [3:0] an,
    output wire [7:0] led
);
    reg [7:0] startup_reset = 8'hff;
    (* ASYNC_REG = "TRUE" *) reg reset_meta = 1'b1, reset_sync = 1'b1;
    (* ASYNC_REG = "TRUE" *) reg [3:0] sw_meta, sw_sync;
    wire rst = (|startup_reset) || reset_sync;
    always @(posedge clk) begin
        startup_reset <= {startup_reset[6:0], 1'b0};
        reset_meta <= btnC;
        reset_sync <= reset_meta;
        if (rst) begin
            sw_meta <= 0;
            sw_sync <= 0;
        end else begin
            sw_meta <= sw;
            sw_sync <= sw_meta;
        end
    end

    wire [9:0] throttle;
    wire rc_valid, motor_ready;
    RadioReceiver #(
        .CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE), .TIMEOUT_MS(TIMEOUT_MS)
    ) radio (
        .clk(clk), .rst(rst), .ibus_rx(tx), .throttle(throttle),
        .roll_command(), .pitch_command(), .yaw_command(),
        .arm_request(), .avoidance_enable(), .frame_valid(),
        .rc_valid(rc_valid), .channels()
    );

    // Each switch independently selects its motor; any nonempty mask is valid.
    wire selection_valid = |sw_sync;
    wire throttle_idle = throttle < 10'd10;
    reg [3:0] accepted_selection;
    reg idle_seen;
    // Each selection must see throttle below 10 after startup, reset, radio loss,
    // or a switch change. No radio arm channel or flight controller is needed.
    always @(posedge clk) begin
        if (rst) begin
            accepted_selection <= 0;
            idle_seen <= 0;
        end else if (!rc_valid || !motor_ready || !selection_valid ||
                     sw_sync != accepted_selection) begin
            accepted_selection <= sw_sync;
            idle_seen <= 0;
        end else if (throttle_idle) idle_seen <= 1;
    end
    wire drive_enabled = !rst && rc_valid && motor_ready && selection_valid &&
                         idle_seen && (sw_sync == accepted_selection);
    // RC 0..9 is stop; 10..1000 maps to ordinary DShot 67..2047.
    // Reserved DShot commands 1..47 can never be produced by this mapping.
    wire [10:0] dshot_throttle = throttle_idle ? 11'd0 :
                                 {throttle, 1'b0} + 11'd47;
    MotorControl #(
        .CLK_FREQ(CLK_FREQ), .DSHOT_RATE(DSHOT_RATE),
        .UPDATE_HZ(MOTOR_UPDATE_HZ), .STARTUP_MS(MOTOR_STARTUP_MS)
    ) motors (
        .clk(clk), .rst(rst), .motors_armed(drive_enabled),
        .command_valid(rc_valid && selection_valid),
        .front_right_throttle(sw_sync[0] ? dshot_throttle : 11'd0),
        .front_left_throttle(sw_sync[1] ? dshot_throttle : 11'd0),
        .back_right_throttle(sw_sync[2] ? dshot_throttle : 11'd0),
        .back_left_throttle(sw_sync[3] ? dshot_throttle : 11'd0),
        .esc_front_right(esc[0]), .esc_front_left(esc[1]),
        .esc_back_right(esc[2]), .esc_back_left(esc[3]),
        .ready(motor_ready), .busy()
    );

    reg [25:0] bcd_shift;
    integer bit_index, digit_index;
    always @* begin
        bcd_shift = {16'b0, throttle};
        for (bit_index = 0; bit_index < 10; bit_index = bit_index + 1) begin
            for (digit_index = 0; digit_index < 4; digit_index = digit_index + 1)
                if (bcd_shift[10 + digit_index*4 +: 4] >= 5)
                    bcd_shift[10 + digit_index*4 +: 4] =
                        bcd_shift[10 + digit_index*4 +: 4] + 4'd3;
            bcd_shift = bcd_shift << 1;
        end
    end
    SevenSegmentDisplay #(
        .CLK_FREQ(CLK_FREQ), .REFRESH_HZ(DISPLAY_REFRESH_HZ), .F_IS_MINUS(1)
    ) display (
        .clk(clk), .rst(rst), .write(1'b1),
        .value(rc_valid ? bcd_shift[25:10] : 16'hffff),
        .digit_enable(4'b1111), .decimal_points(4'b0000),
        .seg(seg), .dp(dp), .an(an)
    );

    localparam integer HEARTBEAT_CYCLES = (CLK_FREQ >= 2) ? CLK_FREQ / 2 : 1;
    localparam integer HEARTBEAT_WIDTH = (HEARTBEAT_CYCLES > 1) ? $clog2(HEARTBEAT_CYCLES) : 1;
    reg [HEARTBEAT_WIDTH-1:0] heartbeat_count;
    reg heartbeat;
    always @(posedge clk) begin
        if (rst) begin
            heartbeat_count <= 0;
            heartbeat <= 0;
        end else if (heartbeat_count == HEARTBEAT_CYCLES - 1) begin
            heartbeat_count <= 0;
            heartbeat <= !heartbeat;
        end else heartbeat_count <= heartbeat_count + 1'b1;
    end
    assign led = {heartbeat, rc_valid, motor_ready, drive_enabled,
                  selection_valid ? sw_sync : 4'b0000};
endmodule
