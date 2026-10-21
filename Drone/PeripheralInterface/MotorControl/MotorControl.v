`timescale 1ns / 1ps

// Four independent ESC channels, updated together on a dedicated periodic tick.
module MotorControl #(
    parameter integer CLK_FREQ = 100_000_000,
    parameter integer DSHOT_RATE = 300_000,
    parameter integer UPDATE_HZ = 1_000,
    parameter integer STARTUP_MS = 1_000
) (
    input wire clk,
    input wire rst,
    input wire motors_armed,
    input wire command_valid, // Must fall on stale commands / loss of RC.
    input wire [10:0] front_right_throttle,
    input wire [10:0] front_left_throttle,
    input wire [10:0] back_right_throttle,
    input wire [10:0] back_left_throttle,
    output wire esc_front_right,
    output wire esc_front_left,
    output wire esc_back_right,
    output wire esc_back_left,
    output wire ready, // Startup stop interval elapsed; not ESC feedback.
    output wire busy
);
    localparam integer BIT_CYCLES = (DSHOT_RATE > 0) ?
        (CLK_FREQ + DSHOT_RATE / 2) / DSHOT_RATE : 1;
    localparam integer STARTUP_FRAMES = (STARTUP_MS * UPDATE_HZ + 999) / 1000;
    localparam integer STARTUP_WIDTH = (STARTUP_FRAMES > 0) ? $clog2(STARTUP_FRAMES + 1) : 1;
    reg [STARTUP_WIDTH-1:0] startup_count;
    wire update_tick;
    wire [3:0] channel_busy;
    wire send = update_tick && !busy;
    wire enable = ready && motors_armed && command_valid;
    assign ready = !rst && (startup_count == STARTUP_FRAMES);
    assign busy = |channel_busy;

    // synthesis translate_off
    initial begin
        if (STARTUP_MS < 0 || UPDATE_HZ <= 0)
            $fatal(1, "MotorControl: require STARTUP_MS >= 0 and UPDATE_HZ > 0");
        if (UPDATE_HZ > 0 && CLK_FREQ / UPDATE_HZ <= 18 * BIT_CYCLES)
            $fatal(1, "MotorControl: update period must exceed packet plus low gap");
    end
    // synthesis translate_on

    TickGenerator #(.CLK_FREQ(CLK_FREQ), .TICK_FREQ(UPDATE_HZ)) update_timer
        (.clk(clk), .rst(rst), .tick(update_tick));

    always @(posedge clk) begin
        if (rst) startup_count <= 0;
        else if (send && !ready) startup_count <= startup_count + 1'b1;
    end

    MotorDriver #(.CLK_FREQ(CLK_FREQ), .DSHOT_RATE(DSHOT_RATE)) motor_front_right (
        .clk(clk), .rst(rst), .send(send), .enable(enable),
        .throttle(front_right_throttle), .esc_signal(esc_front_right), .busy(channel_busy[0])
    );
    MotorDriver #(.CLK_FREQ(CLK_FREQ), .DSHOT_RATE(DSHOT_RATE)) motor_front_left (
        .clk(clk), .rst(rst), .send(send), .enable(enable),
        .throttle(front_left_throttle), .esc_signal(esc_front_left), .busy(channel_busy[1])
    );
    MotorDriver #(.CLK_FREQ(CLK_FREQ), .DSHOT_RATE(DSHOT_RATE)) motor_back_right (
        .clk(clk), .rst(rst), .send(send), .enable(enable),
        .throttle(back_right_throttle), .esc_signal(esc_back_right), .busy(channel_busy[2])
    );
    MotorDriver #(.CLK_FREQ(CLK_FREQ), .DSHOT_RATE(DSHOT_RATE)) motor_back_left (
        .clk(clk), .rst(rst), .send(send), .enable(enable),
        .throttle(back_left_throttle), .esc_signal(esc_back_left), .busy(channel_busy[3])
    );
endmodule
