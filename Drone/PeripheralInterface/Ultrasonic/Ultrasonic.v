`timescale 1ns / 1ps

// HC-SR04 pulse-width ranging. Echo MUST be reduced to 3.3V externally.
// enable=0 disables triggering and invalidates the reading. The bench top
// enables ranging for display/scope use only; hover control is independent.
module Ultrasonic #(
    parameter integer CLK_FREQ = 100_000_000,
    parameter integer PERIOD_US = 65000,
    parameter integer TRIGGER_US = 10,
    parameter integer TRIGGER_ON_STUCK_ECHO = 0, // Scope/bench mode: keep issuing pulses.
    parameter integer TIMEOUT_US = 35000,
    parameter integer MIN_MM = 20,
    parameter integer MAX_MM = 4000
) (
    input wire clk, rst, enable,
    input wire echo,
    output wire trigger,
    output wire busy,
    output reg [15:0] distance_mm,
    output reg sample_valid, measurement_done,
    output wire valid,
    output reg [2:0] fault_code // 0 OK, 1 no rise, 2 stuck high, 3 range, 4 high before trigger
);
    localparam integer PERIOD_CYCLES = (64'd1 * CLK_FREQ * PERIOD_US + 999999)/1000000;
    localparam integer TRIGGER_CYCLES = (64'd1 * CLK_FREQ * TRIGGER_US + 999999)/1000000;
    localparam integer TIMEOUT_CYCLES = (64'd1 * CLK_FREQ * TIMEOUT_US + 999999)/1000000;
    localparam [1:0] IDLE=0, TRIGGER=1, WAIT_ECHO=2, MEASURE=3;
    (* ASYNC_REG = "TRUE" *) reg echo_meta, echo_sync;
    reg [1:0] state;
    reg [31:0] period_count, trigger_count, age_count, echo_count;
    reg measurement_valid;
    // 340m/s reference, round to nearest mm. Constant division; no real arithmetic.
    wire [63:0] converted_mm = (64'd340000 * echo_count + CLK_FREQ)/(64'd2 * CLK_FREQ);
    assign trigger = !rst && enable && state == TRIGGER;
    assign busy = !rst && enable && state != IDLE;
    assign valid = !rst && enable && measurement_valid;

    always @(posedge clk) begin
        if (rst) begin echo_meta <= 0; echo_sync <= 0; end
        else begin echo_meta <= echo; echo_sync <= echo_meta; end
    end
    always @(posedge clk) begin
        if (rst || !enable) begin
            state <= IDLE; period_count <= PERIOD_CYCLES-1;
            trigger_count <= 0; age_count <= 0; echo_count <= 0;
            distance_mm <= 0; sample_valid <= 0; measurement_done <= 0;
            measurement_valid <= 0; fault_code <= 0;
        end else begin
            sample_valid <= 0; measurement_done <= 0;
            if (period_count < PERIOD_CYCLES-1) period_count <= period_count+1'b1;
            case (state)
                IDLE: if (period_count == PERIOD_CYCLES-1) begin
                    period_count <= 0;
                    if (echo_sync && !TRIGGER_ON_STUCK_ECHO) begin
                        measurement_valid <= 0; measurement_done <= 1; fault_code <= 4;
                    end else begin state <= TRIGGER; trigger_count <= 0; end
                end
                TRIGGER: if (trigger_count == TRIGGER_CYCLES-1) begin
                    state <= WAIT_ECHO; age_count <= 0;
                end else trigger_count <= trigger_count+1'b1;
                WAIT_ECHO: begin
                    age_count <= age_count+1'b1;
                    if (echo_sync) begin state <= MEASURE; echo_count <= 1; end
                    else if (age_count >= TIMEOUT_CYCLES-1) begin
                        state <= IDLE; measurement_valid <= 0;
                        measurement_done <= 1; fault_code <= 1;
                    end
                end
                MEASURE: begin
                    age_count <= age_count+1'b1;
                    if (!echo_sync) begin
                        state <= IDLE; measurement_done <= 1;
                        if (converted_mm < MIN_MM || converted_mm > MAX_MM) begin
                            measurement_valid <= 0; fault_code <= 3;
                        end else begin
                            distance_mm <= converted_mm[15:0]; sample_valid <= 1;
                            measurement_valid <= 1; fault_code <= 0;
                        end
                    end else if (age_count >= TIMEOUT_CYCLES-1) begin
                        state <= IDLE; measurement_valid <= 0;
                        measurement_done <= 1; fault_code <= 2;
                    end else echo_count <= echo_count+1'b1;
                end
                default: begin state <= IDLE; measurement_valid <= 0; end
            endcase
        end
    end
    // synthesis translate_off
    initial begin
        if (CLK_FREQ < 1_000_000 || TRIGGER_US < 10 || PERIOD_US <= 60000 ||
            TIMEOUT_US <= 0 || TIMEOUT_US+TRIGGER_US >= PERIOD_US ||
            MIN_MM < 1 || MAX_MM < MIN_MM || MAX_MM > 65535 ||
            PERIOD_CYCLES < 1 || TRIGGER_CYCLES < 1 || TIMEOUT_CYCLES < 1)
            $fatal(1,"Ultrasonic: invalid timing/range parameters");
    end
    // synthesis translate_on
endmodule
