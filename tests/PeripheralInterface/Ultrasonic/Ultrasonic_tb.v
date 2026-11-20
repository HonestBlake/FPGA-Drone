`include "../../../Drone/PeripheralInterface/Ultrasonic/Ultrasonic.v"
`timescale 1ns / 1ps
module Ultrasonic_tb;
    reg clk=0, rst=1, enable=0, echo=0;
    wire trigger, busy, valid, sample_valid, done;
    wire [15:0] mm;
    wire [2:0] fault;
    integer ticks, last_start=-1, start_tick;
    Ultrasonic #(.CLK_FREQ(1_000_000)) dut (
        .clk(clk), .rst(rst), .enable(enable), .echo(echo), .trigger(trigger),
        .busy(busy), .distance_mm(mm), .sample_valid(sample_valid),
        .measurement_done(done), .valid(valid), .fault_code(fault));
    wire scope_trigger;
    integer scope_start=-1, scope_previous=-1, scope_pulses=0;
    Ultrasonic #(.CLK_FREQ(1_000_000), .TRIGGER_US(20), .TRIGGER_ON_STUCK_ECHO(1)) scope_dut(
        .clk(clk), .rst(rst), .enable(enable), .echo(1'b1), .trigger(scope_trigger));
    always @(posedge scope_trigger) begin
        if(scope_previous>=0 && ticks-scope_previous!=65000)
            $fatal(1,"Scope trigger spacing with stuck echo");
        scope_previous=ticks; scope_start=ticks; scope_pulses=scope_pulses+1;
    end
    always @(negedge scope_trigger) begin
        if(!rst && enable && scope_start>=0 && ticks-scope_start!=20)
            $fatal(1,"Scope trigger pulse width");
    end
    always @(negedge enable) begin scope_previous=-1; scope_start=-1; end
    always #5 clk=~clk;
    always @(posedge clk) ticks=ticks+1;
    task clocks;
        input integer n;
        integer k;
        begin for(k=0;k<n;k=k+1) @(negedge clk); end
    endtask
    task start_measurement;
        begin
            @(posedge trigger); @(negedge clk); start_tick=ticks;
            if(last_start>=0 && start_tick-last_start!=65000)
                $fatal(1,"Trigger spacing %0d",start_tick-last_start);
            last_start=start_tick;
            clocks(9);
            if(!trigger) $fatal(1,"Trigger shorter than 10us");
            clocks(1);
            if(trigger) $fatal(1,"Trigger longer than 10us");
        end
    endtask
    task good_echo;
        input integer width_us, expected_mm;
        begin
            start_measurement;
            clocks(100); echo=1; clocks(width_us); echo=0;
            wait(done); @(negedge clk);
            if(!valid || !sample_valid || fault!=0 || mm!=expected_mm)
                $fatal(1,"Range width=%0d mm=%0d fault=%0d",width_us,mm,fault);
            clocks(1);
            if(sample_valid || done) $fatal(1,"Pulse width");
        end
    endtask
    initial begin
        $dumpfile("Ultrasonic.vcd"); $dumpvars(0,Ultrasonic_tb);
        ticks=0; clocks(4); rst=0; clocks(20);
        if(trigger || busy || valid || done) $fatal(1,"Disabled at startup");
        enable=1;
        good_echo(5882,1000);
        good_echo(118,20);
        good_echo(23529,4000);
        // Reject too short and beyond the specified four-meter range.
        start_measurement; clocks(100); echo=1; clocks(50); echo=0;
        wait(done); @(negedge clk);
        if(valid || sample_valid || fault!=3 || mm!=4000) $fatal(1,"Short pulse rejected/hold");
        start_measurement; clocks(100); echo=1; clocks(25000); echo=0;
        wait(done); @(negedge clk);
        if(valid || sample_valid || fault!=3) $fatal(1,"Overrange");
        start_measurement; wait(done); @(negedge clk);
        if(valid || fault!=1 || busy) $fatal(1,"Missing echo timeout");
        good_echo(2941,500);
        start_measurement; clocks(100); echo=1;
        wait(done); @(negedge clk);
        if(valid || fault!=2 || busy) $fatal(1,"Stuck high timeout");
        clocks(65000); // Still high must never retrigger.
        if(trigger || fault!=4 || valid) $fatal(1,"Echo high before trigger");
        echo=0; enable=0; clocks(4); last_start=-1; enable=1;
        good_echo(5882,1000);
        start_measurement; clocks(100); echo=1; clocks(100); enable=0; clocks(2);
        if(trigger || busy || valid || done || sample_valid) $fatal(1,"Disable mid-measurement");
        rst=1; clocks(3);
        if(mm!=0 || fault!=0) $fatal(1,"Reset");
        if(scope_pulses<3) $fatal(1,"Scope pulses stopped on stuck-high echo");
        $display("PASS: Ultrasonic trigger timing/spacing, boundary ranges, asynchronous echo, no/stuck echo, recovery, disable/reset");
        $finish;
    end
    initial begin #15000000; $fatal(1,"Ultrasonic watchdog"); end
endmodule
