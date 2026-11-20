`include "../../../tests/support/XadcModel.vh"
`timescale 1ns/1ps
`include "Icm20948Model.vh"
`timescale 1ns/1ps
module UltrasonicBench_tb;
 reg clk=0,btn=0,echo=0;
 reg [15:0] sw=16'h0400;
 wire trigger,cs,sclk,mosi,miso;
 wire [15:0] led;
 wire [6:0] seg;
 wire [3:0] an;
 wire dp;
 integer count,n,digit;
 reg [3:0] seen;
 reg [6:0] expected;
 always #41.667 clk=~clk;
 BatteryLightsFlightTest #(.CLK_FREQ(12_000_000),.CORE_CLK_DIV(1),
  .IMU_POWERUP_CYCLES(20),.IMU_RESET_CYCLES(20),.IMU_WAKE_CYCLES(20),
  .IMU_CALIBRATION_SAMPLES(4),.DISPLAY_REFRESH_HZ(12000)) dut(
  .clk(clk),.btnC(btn),.tx(1'b1),.sw(sw),.battery_vin_p(1'b0),.battery_vin_n(1'b0),
  .lidar_rx(1'b1),.imu_spi_miso(miso),.imu_spi_cs_n(cs),.imu_spi_sclk(sclk),.imu_spi_mosi(mosi),
  .ultrasonic_echo(echo),.ultrasonic_trigger(trigger),.led(led),.seg(seg),.an(an),.dp(dp));
 Icm20948Model sensor(dut.rst,cs,sclk,mosi,miso);
 initial begin #150_000_000; $fatal(1,"Ultrasonic top watchdog");end
 initial begin
  @(posedge trigger); count=0;
  while(trigger) begin @(negedge clk);count=count+1;end
  if(count<239 || count>241) $fatal(1,"Trigger not 20us");
  #10000;echo=1;#3600000;echo=0;
  wait(dut.ultrasonic_valid);
  if(dut.ultrasonic_distance_mm!=612) $fatal(1,"Distance scale");
  wait(dut.imu_calibrated);
  repeat(2200) @(negedge clk);
  if(dut.display_value!==16'h0612 || led[10]!==1 || dut.fc_armed)
   $fatal(1,"SW10 range/health/control isolation");
  seen=0;
  for(n=0;n<1100;n=n+1) begin
   @(negedge clk);
   if(an!=15) begin
    case(an) 14:digit=0;13:digit=1;11:digit=2;7:digit=3;default:$fatal(1,"anode");endcase
    case(digit) 0:expected=7'h24;1:expected=7'h79;2:expected=7'h02;3:expected=7'h40;endcase
    seen[digit]=1;
    if(seg!==expected || dp!==1) $fatal(1,"mm display physical glyph");
   end
  end
  if(seen!=15) $fatal(1,"Missing mm digits");
  sw[14]=1;repeat(6) @(negedge clk);
  if(dut.display_value!==16'he010) $fatal(1,"Error view must override distance");
  sw[14]=0;repeat(6) @(negedge clk);
  if(dut.display_value!==16'h0612) $fatal(1,"Distance view restore");
  @(posedge trigger);wait(dut.ultrasonic_done);
  #1;if(dut.ultrasonic_valid || dut.ultrasonic_fault!=1) $fatal(1,"Missing echo handling");
  repeat(3) @(negedge clk);
  if(dut.display_value!==16'hffff) $fatal(1,"Invalid range must show dashes");
  force dut.heartbeat=0;#1;if(led[10]!==0) $fatal(1,"Ultrasonic unhealthy blink off");
  force dut.heartbeat=1;#1;if(led[10]!==1) $fatal(1,"Ultrasonic unhealthy blink on");
  release dut.heartbeat;
  $display("PASS: physical ultrasonic trigger/echo, 612mm display, healthy/stale status, error-view priority, no hover control");
  $finish;
 end
endmodule
