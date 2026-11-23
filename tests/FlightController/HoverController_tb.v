`include "../../Drone/FlightController/HoverController.v"
`timescale 1ns/1ps
module HoverController_tb;
 reg clk=0,rst=1,enable=1,armed=1,control_valid=1,attitude_ok=1;
 reg sample_valid=0,range_valid=1,tick=1;
 reg [15:0] distance_mm=600;
 reg [9:0] manual_throttle=500;
 wire active,fresh,fault,pickup,blend;
 wire [9:0] out;
 wire [15:0] target;
 wire signed [15:0] speed;
 wire [9:0] derivative_out;
 wire signed [15:0] derivative_speed;
 HoverController #(.CLK_FREQ(1000),.CONTROL_HZ(1000),.HANDOVER_MS(20),.KP(0)) derivative(
  .clk(clk),.rst(rst),.enable(enable),.armed(armed),.control_valid(control_valid),.attitude_ok(attitude_ok),
  .sample_valid(sample_valid),.range_valid(range_valid),.distance_mm(distance_mm),.manual_throttle(manual_throttle),
  .control_tick(tick),.throttle_out(derivative_out),.vertical_speed_mmps(derivative_speed));
 integer i,old;
 always #5 clk=~clk;
 HoverController #(.CLK_FREQ(1000),.CONTROL_HZ(1000),.HANDOVER_MS(20),.KP(256),.KD(0)) dut(
  .clk(clk),.rst(rst),.enable(enable),.armed(armed),.control_valid(control_valid),.attitude_ok(attitude_ok),
  .sample_valid(sample_valid),.range_valid(range_valid),.distance_mm(distance_mm),.manual_throttle(manual_throttle),
  .active(active),.range_fresh(fresh),.throttle_out(out),.target_height_mm(target),.vertical_speed_mmps(speed),
  .fault(fault),.control_tick(tick),.pickup_wait(pickup),.handover_active(blend));
 task clocks;input integer n;begin repeat(n)@(negedge clk);end endtask
 task sample;input [15:0] h;begin distance_mm=h;sample_valid=1;clocks(1);sample_valid=0;end endtask
 task restart;
  begin rst=1;enable=1;armed=1;control_valid=1;attitude_ok=1;range_valid=1;manual_throttle=500;tick=1;sample_valid=0;
   clocks(3);rst=0;sample(600);clocks(2);end
 endtask
 task engage;
  begin enable=0;clocks(2);sample(600);enable=1;clocks(2);
   if(!active || fault || target!=600 || out!=manual_throttle)$fatal(1,"Target/base capture");end
 endtask
 initial begin
  $dumpfile("HoverController.vcd");$dumpvars(0,HoverController_tb);
  restart;if(active || out!=500)$fatal(1,"Enable at boot");
  manual_throttle=0;enable=0;clocks(2);enable=1;clocks(2);
  if(active || !fault || out!=0)$fatal(1,"Must not start idle motors");
  restart;enable=0;clocks(2);sample(120);enable=1;clocks(2);
  if(!active || target!=120 || out!=500)$fatal(1,"Ground sensor offset must be captured without zeroing");
  restart;enable=0;clocks(2);sample(2001);enable=1;clocks(2);
  if(active || !fault || fresh)$fatal(1,"Out-of-window entry accepted");
  restart;engage;manual_throttle=0;sample(2001);
  if(!active || !fault || out!=500)$fatal(1,"Out-of-window hold dropped collective");
  restart;engage;sample(500);if(derivative_out!=596 || derivative_speed!=-384)$fatal(1,"Filtered descent damping");if(out!=600)$fatal(1,"Low height must increase throttle");
  sample(700);if(derivative_out!=380 || derivative_speed!=481)$fatal(1,"Filtered ascent damping");if(out!=400 || target!=600)$fatal(1,"High height must decrease throttle");
  sample(400);if(out!=650)$fatal(1,"Upper correction clamp");sample(700);if(out!=400)$fatal(1,"Correction recovery");
  sample(600);manual_throttle=0;clocks(2);
  if(!active || out!=500)$fatal(1,"Low stick cancelled height hold");
  enable=0;clocks(2);if(!active || !pickup || out!=500)$fatal(1,"OFF at zero cut throttle");
  sample(550);if(out!=550 || !pickup)$fatal(1,"Height loop stopped while awaiting pickup");
  manual_throttle=549;clocks(2);if(!pickup)$fatal(1,"Pickup below current holding command");
  manual_throttle=550;tick=0;clocks(2);if(!blend || out!=550)$fatal(1,"Bumpless pickup equality");
  // Drop the stick immediately after pickup. It must ramp down, not cut.
  manual_throttle=0;tick=1;old=550;
  for(i=0;i<19;i=i+1)begin clocks(1);if(out>=old || old-out>28 || !active)$fatal(1,"Signed downward handover %d",out);old=out;end
  clocks(1);if(active || out!=0)$fatal(1,"Handover did not finish at manual zero");
  // OFF with an already higher stick ramps upward.
  restart;engage;manual_throttle=800;tick=0;enable=0;clocks(2);
  if(!blend || out!=500)$fatal(1,"High stick jumped at OFF");tick=1;clocks(10);
  if(out!=650 || !blend)$fatal(1,"Halfway interpolation");clocks(10);if(active || out!=800)$fatal(1,"Upward handover");
  // Re-engage while awaiting pickup: capture current height/output, not zero stick.
  restart;engage;manual_throttle=0;enable=0;clocks(2);sample(650);old=out;
  enable=1;clocks(2);if(!active || pickup || target!=650 || out!=old || fault)$fatal(1,"Recapture during pickup");
  // Invalid/outlier/stale range freezes collective and still permits pickup.
  range_valid=0;clocks(2);if(!active || !fault || out!=old)$fatal(1,"Range loss cut collective");
  enable=0;clocks(2);manual_throttle=old;tick=0;clocks(2);
  if(!blend || out!=old)$fatal(1,"Fault blocked manual pickup");tick=1;clocks(20);if(active)$fatal(1,"Fault pickup did not finish");
  restart;engage;manual_throttle=0;sample(1000);
  if(!active || !fault || out!=500)$fatal(1,"Outlier changed thrust");
  restart;engage;manual_throttle=0;clocks(151);
  if(!active || fresh || !fault || out!=500)$fatal(1,"Stale sensor did not freeze collective");
  restart;engage;manual_throttle=0;attitude_ok=0;clocks(2);
  if(!active || !fault || out!=500)$fatal(1,"Tilt should freeze height feedback, not drop thrust");
  restart;engage;manual_throttle=0;armed=0;#1;if(active || out!=0)$fatal(1,"Disarm must immediately bypass hold");
  clocks(2);armed=1;sample(600);clocks(2);if(active)$fatal(1,"Hold restarted after rearm");
  restart;engage;control_valid=0;#1;if(active)$fatal(1,"RC/IMU/control health gating");
  clocks(2);control_valid=1;clocks(2);if(active)$fatal(1,"Health recovery restarted hold");
  restart;manual_throttle=1023;enable=0;clocks(2);if(out!=1000)$fatal(1,"Manual clamp");
  rst=1;clocks(2);if(out!=0 || active || fault)$fatal(1,"Reset");
  $display("PASS: hover capture, hold at zero stick, live-command pickup, signed smooth handover, recapture, range/tilt freeze, health/disarm and reset");$finish;
 end
 initial begin #200000;$fatal(1,"Hover watchdog");end
endmodule
