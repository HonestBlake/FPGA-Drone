`timescale 1ns / 1ps
// Ultrasonic collective hold plus switch-OFF throttle pickup. No arming or ESC
// outputs. RC/IMU/disarm gating belongs to the existing flight controller.
module HoverController #(
    parameter integer CLK_FREQ=100_000_000, CONTROL_HZ=1000,
    parameter integer RANGE_TIMEOUT_MS=150, SAMPLE_PERIOD_MS=65,
    parameter integer MIN_HEIGHT_MM=80, MAX_HEIGHT_MM=2000,
    parameter integer MIN_ENGAGE_THROTTLE=100, IDLE_THROTTLE=10,
    parameter integer KP=16, KD=64, MAX_CORRECTION=150, MAX_STEP_MM=300,
    parameter integer HANDOVER_MS=500
)(
    input wire clk,rst,enable,armed,control_valid,attitude_ok,
    input wire sample_valid,range_valid,
    input wire [15:0] distance_mm,
    input wire [9:0] manual_throttle,
    output wire active,range_fresh,
    output wire [9:0] throttle_out,
    output reg [15:0] target_height_mm,
    output reg signed [15:0] vertical_speed_mmps,
    output reg fault,
    input wire control_tick,
    output wire pickup_wait,handover_active
);
    localparam integer TIMEOUT_CYCLES=(64'd1*CLK_FREQ*RANGE_TIMEOUT_MS+999)/1000;
    localparam integer BLEND_TICKS=(CONTROL_HZ*HANDOVER_MS+999)/1000;
    localparam integer BLEND_WIDTH=(BLEND_TICKS>1)?$clog2(BLEND_TICKS+1):1;
    localparam [1:0] MANUAL=0,HOLD=1,PICKUP=2,BLEND=3;
    reg [1:0] state;
    reg [31:0] age;
    reg seen,enable_previous;
    reg [15:0] last_height;
    reg [9:0] base_throttle,held_throttle,blend_start;
    reg [BLEND_WIDTH-1:0] blend_count;
    integer delta,speed,filtered_speed,error_mm,correction,candidate,blend_value;
    reg signed [10:0] blend_difference;
    reg signed [BLEND_WIDTH+1:0] blend_step;
    reg signed [BLEND_WIDTH+12:0] blend_product;
    wire inside_range=distance_mm>=MIN_HEIGHT_MM && distance_mm<=MAX_HEIGHT_MM;
    wire [9:0] manual_clamped=manual_throttle>1000 ? 10'd1000 : manual_throttle;
    assign range_fresh=!rst && seen && age<TIMEOUT_CYCLES && range_valid;
    wire measurement_usable=attitude_ok && range_fresh && (!sample_valid || inside_range);
    assign active=!rst && state!=MANUAL && armed && control_valid;
    assign pickup_wait=active && state==PICKUP;
    assign handover_active=active && state==BLEND;
    assign throttle_out=rst ? 10'd0 : active ? held_throttle : manual_clamped;
    always @* begin
        delta=$signed({1'b0,distance_mm})-$signed({1'b0,last_height});
        speed=delta*1000/SAMPLE_PERIOD_MS;
        if(speed>32767)speed=32767;
        if(speed< -32768)speed=-32768;
        // Low-pass the derivative to reduce thrust changes from mm-scale noise.
        filtered_speed=($signed(vertical_speed_mmps)*3+speed)/4;
        error_mm=$signed({1'b0,target_height_mm})-$signed({1'b0,distance_mm});
        correction=(error_mm*KP-filtered_speed*KD)/256;
        if(correction>MAX_CORRECTION)correction=MAX_CORRECTION;
        if(correction< -MAX_CORRECTION)correction=-MAX_CORRECTION;
        candidate=base_throttle+correction;
        if(candidate<IDLE_THROTTLE)candidate=IDLE_THROTTLE;
        if(candidate>1000)candidate=1000;
        // Interpolate against the live stick; a subsequent drop to zero also
        // ramps during handover rather than bypassing pickup with a sudden cut.
        blend_step=blend_count+1;
        blend_difference=$signed({1'b0,manual_clamped})-$signed({1'b0,blend_start});
        blend_product=blend_difference*blend_step;
        blend_value=$signed({1'b0,blend_start})+blend_product/BLEND_TICKS;
        if(blend_value<0)blend_value=0;
        if(blend_value>1000)blend_value=1000;
    end
    always @(posedge clk) begin
        if(rst) begin
            state<=MANUAL;age<=TIMEOUT_CYCLES;seen<=0;enable_previous<=1;
            last_height<=0;base_throttle<=0;held_throttle<=0;blend_start<=0;blend_count<=0;
            target_height_mm<=0;vertical_speed_mmps<=0;fault<=0;
        end else begin
            enable_previous<=enable;
            if(age<TIMEOUT_CYCLES)age<=age+1'b1;
            if(!range_valid || (sample_valid && !inside_range))seen<=0;
            if(sample_valid && range_valid && inside_range) begin
                last_height<=distance_mm;age<=0;seen<=1;
            end
            // Health/arm losses never leave a latched autonomous throttle for
            // the next arm. Switch OFF then ON is required to re-enter hold.
            if(!armed || !control_valid) begin
                state<=MANUAL;vertical_speed_mmps<=0;
                if(state!=MANUAL || (enable && !enable_previous))fault<=1;
                else if(!enable)fault<=0;
            end else case(state)
                MANUAL: begin
                    vertical_speed_mmps<=0;
                    if(!enable)fault<=0;
                    else if(!enable_previous) begin
                        if(measurement_usable && manual_clamped>=MIN_ENGAGE_THROTTLE) begin
                            state<=HOLD;fault<=0;
                            target_height_mm<=sample_valid ? distance_mm : last_height;
                            base_throttle<=manual_clamped;held_throttle<=manual_clamped;
                        end else fault<=1;
                    end
                end
                HOLD,PICKUP: begin
                    if(!enable && manual_clamped>=held_throttle) begin
                        state<=BLEND;blend_start<=held_throttle;blend_count<=0;
                    end else if(enable && !enable_previous) begin
                        // Re-enable during pending pickup: capture a new height
                        // using the current output, even if the stick is zero.
                        if(measurement_usable) begin
                            state<=HOLD;fault<=0;vertical_speed_mmps<=0;
                            target_height_mm<=sample_valid ? distance_mm : last_height;
                            base_throttle<=held_throttle;
                        end else fault<=1;
                    end else begin
                        if(!enable)state<=PICKUP;
                        // Range loss, spikes or excessive tilt freeze the last
                        // collective, not zero/manual-low. Pickup remains usable.
                        if(!measurement_usable)fault<=1;
                        else if(sample_valid && !fault) begin
                            if(delta>MAX_STEP_MM || delta< -MAX_STEP_MM)fault<=1;
                            else begin held_throttle<=candidate[9:0];vertical_speed_mmps<=filtered_speed;end
                        end
                    end
                end
                BLEND: begin
                    if(enable && !enable_previous) begin
                        if(measurement_usable) begin
                            state<=HOLD;fault<=0;vertical_speed_mmps<=0;
                            target_height_mm<=sample_valid ? distance_mm : last_height;
                            base_throttle<=held_throttle;
                        end else fault<=1;
                    end else if(control_tick) begin
                        held_throttle<=blend_value[9:0];
                        if(blend_count==BLEND_TICKS-1) begin state<=MANUAL;fault<=0;end
                        else blend_count<=blend_count+1'b1;
                    end
                end
            endcase
        end
    end
    // synthesis translate_off
    initial if(CLK_FREQ<1000 || CONTROL_HZ<1 || RANGE_TIMEOUT_MS<=SAMPLE_PERIOD_MS || SAMPLE_PERIOD_MS<1 ||
        MIN_HEIGHT_MM<1 || MAX_HEIGHT_MM<=MIN_HEIGHT_MM || MAX_HEIGHT_MM>65535 ||
        IDLE_THROTTLE<1 || MIN_ENGAGE_THROTTLE<IDLE_THROTTLE || MIN_ENGAGE_THROTTLE>1000 ||
        MAX_CORRECTION<0 || MAX_CORRECTION>1000 || MAX_STEP_MM<1 || KP<0 || KP>32767 || KD<0 || KD>32767 ||
        HANDOVER_MS<1 || BLEND_TICKS<1 || TIMEOUT_CYCLES<1)
        $fatal(1,"HoverController: invalid timing/limits/gains");
    // synthesis translate_on
endmodule
