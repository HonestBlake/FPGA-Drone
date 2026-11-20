`include "../support/XadcModel.vh"
`include "../../Drone/System/BatteryMonitor.v"
`timescale 1ns / 1ps
module BatteryMonitor_tb;
    reg clk = 0, rst = 1;
    wire [11:0] raw;
    wire [15:0] mv;
    wire [6:0] percent;
    wire valid, sample_valid;
    wire [9:0] tenths;
    integer code, expected_mv, expected_percent, previous_percent;
    BatteryMonitor #(.FILTER_SAMPLES(1), .UPDATE_HZ(100_000_000)) dut (
        .clk(clk), .rst(rst), .battery_vin_p(1'b1), .battery_vin_n(1'b0),
        .battery_raw(raw), .battery_mv(mv), .battery_percent(percent),
        .battery_percent_tenths(tenths), .battery_valid(valid), .battery_sample_valid(sample_valid)
    );
    // Non-default channel and endpoints exercise configuration arithmetic.
    wire [6:0] custom_percent;
    BatteryMonitor #(.CLK_FREQ(50_000_000), .XADC_CHANNEL(14), .FILTER_SAMPLES(1), .UPDATE_HZ(50_000_000),
        .EMPTY_MV(10000), .FULL_MV(12000)) custom (
        .clk(clk), .rst(rst), .battery_vin_p(1'b1), .battery_vin_n(1'b0),
        .battery_percent(custom_percent)
    );
    wire [11:0] smooth_raw;
    wire smooth_valid, smooth_sample;
    reg smooth_done=0;
    integer smooth_cycles=0, smooth_last=-1, smooth_n;
    reg [11:0] smooth_previous=0;
    BatteryMonitor #(.CLK_FREQ(1000), .FILTER_SAMPLES(4), .UPDATE_HZ(10)) smooth(
        .clk(clk), .rst(rst), .battery_vin_p(1'b1), .battery_vin_n(1'b0),
        .battery_raw(smooth_raw), .battery_valid(smooth_valid), .battery_sample_valid(smooth_sample));
    always @(posedge clk) begin
        #1;
        if(rst) begin smooth_cycles=0; smooth_last=-1; smooth_previous=0; end
        else begin
            smooth_cycles=smooth_cycles+1;
            if(smooth_sample) begin
                if(smooth_last>=0 && smooth_cycles-smooth_last!=100)
                    $fatal(1,"Battery publication cadence");
                smooth_last=smooth_cycles;
            end else if(smooth_raw!==smooth_previous)
                $fatal(1,"Battery output changed between publication ticks");
            smooth_previous=smooth_raw;
        end
    end
    initial begin
        smooth.xadc.sample_code=3000;
        wait(!rst); repeat(300) @(negedge clk);
        if(!smooth_valid || smooth_raw!=3000) $fatal(1,"Filtered startup");
        for(smooth_n=0;smooth_n<40;smooth_n=smooth_n+1) begin
            smooth.xadc.sample_code=smooth_n[0] ? 4000 : 2000;
            repeat(16) @(negedge clk);
        end
        if(smooth_raw!=3000) $fatal(1,"Alternating ADC noise did not average to midpoint");
        smooth_done=1;
    end
    always #5 clk = ~clk;
    initial begin
        $dumpfile("BatteryMonitor.vcd"); $dumpvars(0, BatteryMonitor_tb);
        repeat (3) @(negedge clk);
        if ({raw, mv, percent, valid, sample_valid} !== 0) $fatal(1, "Reset");
        if (dut.xadc.INIT_40 !== 16'h3116 || dut.xadc.INIT_41 !== 16'h31af ||
            dut.xadc.INIT_42 !== 16'h0400 || dut.aux_p !== 16'h0040 || dut.aux_n !== 0 ||
            custom.xadc.INIT_40 !== 16'h311e || custom.xadc.INIT_42 !== 16'h0200 ||
            custom.aux_p !== 16'h4000) $fatal(1, "XADC configuration/pin routing");
        rst = 0;
        previous_percent = 0;
        // Exhaustive sweep: scaling, rounding, clamping, monotonicity, sample hold.
        for (code = 0; code < 4096; code = code + 1) begin
            dut.xadc.sample_code = code;
            custom.xadc.sample_code = code;
            // Allow any in-flight read of the previous sample to drain.
            repeat (24) @(negedge clk);
            wait (sample_valid); @(negedge clk);
            expected_mv = (code * 16000 + 2048) / 4096;
            if (expected_mv <= 9000) expected_percent = 0;
            else if (expected_mv >= 12600) expected_percent = 100;
            else expected_percent = ((expected_mv - 9000)*100 + 1800)/3600;
            if (raw !== code || mv !== expected_mv || percent !== expected_percent || valid !== 1)
                $fatal(1, "Conversion code=%0d mv=%0d percent=%0d", code, mv, percent);
            if(expected_mv<=9000) expected_percent=0;
            else if(expected_mv>=12600) expected_percent=1000;
            else expected_percent=((expected_mv-9000)*1000+1800)/3600;
            if(tenths!==expected_percent) $fatal(1,"Tenths percentage");
            if (percent < previous_percent) $fatal(1, "Nonmonotonic");
            previous_percent = percent;
            if (expected_mv <= 10000) expected_percent = 0;
            else if (expected_mv >= 12000) expected_percent = 100;
            else expected_percent = ((expected_mv-10000)*100+1000)/2000;
            if (custom_percent !== expected_percent) $fatal(1, "Custom endpoints");
            @(negedge clk);
            if (mv !== expected_mv) $fatal(1, "Sample pulse/hold");
        end
        if(!smooth_done) $fatal(1,"Filter test incomplete");
        rst = 1; @(negedge clk);
        if ({raw, mv, percent, valid, sample_valid} !== 0) $fatal(1, "Reset after conversion");
        rst = 0; @(negedge clk);
        if (valid !== 0) $fatal(1, "Valid before first new conversion");
        $display("PASS: BatteryMonitor all 4096 codes, endpoints, DRP, configuration, reset");
        $finish;
    end
    initial begin #3000000; $fatal(1, "BatteryMonitor watchdog"); end
endmodule
