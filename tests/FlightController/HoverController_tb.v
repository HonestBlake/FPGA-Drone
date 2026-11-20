`include "../../Drone/FlightController/HoverController.v"
`timescale 1ns / 1ps
module HoverController_tb;
    reg clk=0, rst=1, enable=1, armed=1, control_valid=1, attitude_ok=1;
    reg sample_valid=0, range_valid=1;
    reg [15:0] distance_mm=1000;
    reg [9:0] manual_throttle=500;
    wire active, fresh, fault;
    wire [9:0] throttle_out;
    wire [15:0] target;
    wire signed [15:0] speed;
    HoverController #(.CLK_FREQ(1000)) dut (
        .clk(clk), .rst(rst), .enable(enable), .armed(armed),
        .control_valid(control_valid), .attitude_ok(attitude_ok),
        .sample_valid(sample_valid), .range_valid(range_valid), .distance_mm(distance_mm),
        .manual_throttle(manual_throttle), .active(active), .range_fresh(fresh),
        .throttle_out(throttle_out), .target_height_mm(target),
        .vertical_speed_mmps(speed), .fault(fault));
    always #5 clk=~clk;
    task clocks;
        input integer n; integer k;
        begin for(k=0;k<n;k=k+1) @(negedge clk); end
    endtask
    task sample;
        input [15:0] height;
        begin distance_mm=height; sample_valid=1; clocks(1); sample_valid=0; end
    endtask
    task engage;
        begin
            enable=0; clocks(2); sample(1000); clocks(1); enable=1; clocks(1);
            if(!active || fault || target!=1000 || throttle_out!=manual_throttle)
                $fatal(1,"Bumpless enable/target capture");
        end
    endtask
    initial begin
        $dumpfile("HoverController.vcd"); $dumpvars(0,HoverController_tb);
        clocks(3); if(throttle_out!=0 || active) $fatal(1,"Reset");
        rst=0; sample(1000); clocks(2);
        if(active || throttle_out!=500) $fatal(1,"Must not enable automatically at boot");
        engage;
        clocks(60); sample(990); clocks(1);
        if(throttle_out<=500 || speed>=0) $fatal(1,"Falling must increase thrust");
        clocks(60); sample(1010); clocks(1);
        if(throttle_out>=500 || speed<=0 || target!=1000) $fatal(1,"Rising must reduce thrust/hold target");
        // Extreme accepted step saturates correction, never wraps.
        clocks(60); sample(710); clocks(1);
        if(throttle_out!=650) $fatal(1,"Positive correction limit");
        clocks(60); sample(1010); clocks(1);
        if(throttle_out!=350) $fatal(1,"Negative correction limit");
        sample(1500); clocks(1);
        if(active || !fault || throttle_out!=500) $fatal(1,"Reject implausible step");
        sample(1000); clocks(3);
        if(active) $fatal(1,"No automatic restart after fault");
        engage; range_valid=0; #1;
        if(active || throttle_out!=500) $fatal(1,"Immediate invalid-range gating");
        clocks(2); range_valid=1; sample(1000); clocks(2);
        if(active || !fault) $fatal(1,"Invalid range latches until enable cycle");
        engage; clocks(151);
        if(active || fresh || !fault || throttle_out!=500) $fatal(1,"Stale sensor watchdog");
        engage; attitude_ok=0; clocks(1);
        if(active || !fault) $fatal(1,"Tilt inhibit");
        attitude_ok=1; engage; armed=0; clocks(1);
        if(active || !fault) $fatal(1,"Disarm inhibit");
        armed=1; engage; control_valid=0; clocks(1);
        if(active || !fault) $fatal(1,"Control validity inhibit");
        control_valid=1; engage; manual_throttle=0; #1;
        if(active || throttle_out!=0) $fatal(1,"Pilot idle overrides hover immediately");
        clocks(2); manual_throttle=50;
        enable=0; clocks(1); sample(1000); enable=1; clocks(1);
        if(active || !fault) $fatal(1,"Reject engagement below minimum throttle");
        manual_throttle=950; engage; sample(700); clocks(1);
        if(throttle_out!=1000) $fatal(1,"Upper throttle clamp");
        manual_throttle=100; engage; sample(1300); clocks(1);
        if(throttle_out!=10) $fatal(1,"Lower active throttle clamp");
        enable=0; manual_throttle=1023; clocks(1);
        if(active || fault || throttle_out!=1000) $fatal(1,"Manual clamp/disable");
        manual_throttle=500; range_valid=1; sample(10); enable=1; clocks(1);
        if(active || !fault || fresh) $fatal(1,"Reject out-of-window sample at entry");
        manual_throttle=500; engage; sample(2001); clocks(1);
        if(active || !fault || fresh) $fatal(1,"Reject out-of-window sample while holding");
        enable=0; clocks(1); range_valid=0; clocks(1); enable=1; clocks(1);
        if(active || !fault) $fatal(1,"Reject enable without valid range");
        rst=1; clocks(2);
        if(active || fault || throttle_out!=0 || target!=0) $fatal(1,"Reset after holding");
        $display("PASS: HoverController explicit engagement, target/base capture, PD signs/limits, spike rejection, stale/invalid/tilt/arm/control/idle interlocks and reset");
        $finish;
    end
    initial begin #100000; $fatal(1,"HoverController watchdog"); end
endmodule
