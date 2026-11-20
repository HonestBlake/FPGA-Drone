`timescale 1ns / 1ps

// Standalone experimental height hold in RC throttle units (0..1000).
// Intentionally NOT connected to FlightController or any hardware arm/mode input.
// No motor outputs: a future integration selects throttle_out only when active.
// Gains are provisional, not flight tuned. A dropout returns manual throttle;
// this is NOT an autonomous landing or an instruction to cut motor power.
module HoverController #(
    parameter integer CLK_FREQ = 100_000_000,
    parameter integer RANGE_TIMEOUT_MS = 150,
    parameter integer SAMPLE_PERIOD_MS = 65,
    parameter integer MIN_HEIGHT_MM = 80,
    parameter integer MAX_HEIGHT_MM = 2000,
    parameter integer MIN_ENGAGE_THROTTLE = 100,
    parameter integer IDLE_THROTTLE = 10,
    parameter integer KP = 16, // throttle counts/mm, scaled by 256
    parameter integer KD = 64, // throttle counts/(mm/s), scaled by 256
    parameter integer MAX_CORRECTION = 150,
    parameter integer MAX_STEP_MM = 300
) (
    input wire clk, rst,
    input wire enable, armed, control_valid, attitude_ok,
    input wire sample_valid, range_valid,
    input wire [15:0] distance_mm,
    input wire [9:0] manual_throttle,
    output wire active, range_fresh,
    output wire [9:0] throttle_out,
    output reg [15:0] target_height_mm,
    output reg signed [15:0] vertical_speed_mmps,
    output reg fault // Latches on rejected entry or loss while enabled; OFF clears.
);
    localparam integer TIMEOUT_CYCLES = (64'd1 * CLK_FREQ * RANGE_TIMEOUT_MS+999)/1000;
    reg [31:0] age;
    reg seen, enable_previous, holding;
    reg [15:0] last_height;
    reg [9:0] base_throttle, held_throttle;
    integer delta, speed, error_mm, correction, candidate;
    wire inside_range = distance_mm >= MIN_HEIGHT_MM && distance_mm <= MAX_HEIGHT_MM;
    wire health = armed && control_valid && attitude_ok && range_valid;
    assign range_fresh = !rst && seen && age < TIMEOUT_CYCLES && range_valid;
    assign active = !rst && enable && holding && !fault && health && range_fresh &&
                    manual_throttle >= IDLE_THROTTLE;
    wire [9:0] manual_clamped = manual_throttle > 1000 ? 10'd1000 : manual_throttle;
    assign throttle_out = rst ? 10'd0 : active ? held_throttle : manual_clamped;

    always @* begin
        delta = $signed({1'b0,distance_mm}) - $signed({1'b0,last_height});
        // Samples must arrive at the paired sensor's fixed 65ms start interval.
        // If a measurement is rejected, holding disengages rather than using a
        // derivative based on a skipped interval.
        speed = (delta * 1000) / SAMPLE_PERIOD_MS;
        if (speed > 32767) speed = 32767;
        if (speed < -32768) speed = -32768;
        error_mm = $signed({1'b0,target_height_mm}) - $signed({1'b0,distance_mm});
        correction = (error_mm * KP - speed * KD) / 256;
        if (correction > MAX_CORRECTION) correction = MAX_CORRECTION;
        if (correction < -MAX_CORRECTION) correction = -MAX_CORRECTION;
        candidate = base_throttle + correction;
        if (candidate < IDLE_THROTTLE) candidate = IDLE_THROTTLE;
        if (candidate > 1000) candidate = 1000;
    end
    always @(posedge clk) begin
        if (rst) begin
            age <= TIMEOUT_CYCLES; seen <= 0; enable_previous <= 1;
            holding <= 0; fault <= 0; last_height <= 0;
            base_throttle <= 0; held_throttle <= 0;
            target_height_mm <= 0; vertical_speed_mmps <= 0;
        end else begin
            enable_previous <= enable;
            if (age < TIMEOUT_CYCLES) age <= age+1'b1;
            if (!range_valid || (sample_valid && !inside_range)) seen <= 0;
            if (sample_valid && range_valid && inside_range) begin
                last_height <= distance_mm; age <= 0; seen <= 1;
            end
            if (!enable) begin holding <= 0; fault <= 0; vertical_speed_mmps <= 0; end
            else if (!enable_previous) begin
                // Capture target/base from this measurement if present, otherwise
                // from the latest accepted sample. No enable-at-boot auto capture.
                if (health && range_fresh && !fault &&
                    manual_throttle >= MIN_ENGAGE_THROTTLE &&
                    (!sample_valid || inside_range)) begin
                    holding <= 1;
                    target_height_mm <= sample_valid ? distance_mm : last_height;
                    base_throttle <= manual_clamped; held_throttle <= manual_clamped;
                    vertical_speed_mmps <= 0;
                end else begin holding <= 0; fault <= 1; end
            end else if (holding) begin
                if (!health || !range_fresh || manual_throttle < IDLE_THROTTLE) begin
                    holding <= 0; fault <= 1;
                end else if (sample_valid) begin
                    if (!inside_range || delta > MAX_STEP_MM || delta < -MAX_STEP_MM) begin
                        holding <= 0; fault <= 1;
                    end else begin
                        held_throttle <= candidate[9:0]; vertical_speed_mmps <= speed;
                    end
                end
            end
        end
    end
    // synthesis translate_off
    initial begin
        if (CLK_FREQ < 1000 || RANGE_TIMEOUT_MS <= SAMPLE_PERIOD_MS || SAMPLE_PERIOD_MS <= 0 ||
            MIN_HEIGHT_MM < 1 || MAX_HEIGHT_MM <= MIN_HEIGHT_MM || MAX_HEIGHT_MM > 65535 ||
            IDLE_THROTTLE < 1 || MIN_ENGAGE_THROTTLE < IDLE_THROTTLE || MIN_ENGAGE_THROTTLE > 1000 ||
            MAX_CORRECTION < 0 || MAX_CORRECTION > 1000 || MAX_STEP_MM < 1 ||
            KP < 0 || KP > 32767 || KD < 0 || KD > 32767 || TIMEOUT_CYCLES < 1)
            $fatal(1,"HoverController: invalid limits/gains/timing");
    end
    // synthesis translate_on
endmodule
