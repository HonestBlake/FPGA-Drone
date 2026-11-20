`include "../../Drone/FlightController/PositionEstimator.v"
`timescale 1ns/1ps
module PositionEstimator_tb;
 reg clk=0,rst=1,enable=0,sample_valid=0,healthy=1,clipped=0;
 reg signed[15:0] ax=0,ay=0,az=16384,roll=0,pitch=0,rate=0;
 wire calibrated,degraded,valid;
 wire signed[31:0] x,y,z,vx,vy,vz;
 integer n;
 always #5 clk=~clk;
 PositionEstimator #(.CLK_FREQ(10000),.SAMPLE_HZ(100),.CALIBRATION_SAMPLES(4),.MAX_TRACK_SECONDS(2)) dut(
  .clk(clk),.rst(rst),.enable(enable),.sample_valid(sample_valid),.imu_valid(healthy),.accel_clipped(clipped),
  .accel_x(ax),.accel_y(ay),.accel_z(az),.roll_cd(roll),.pitch_cd(pitch),.yaw_rate_cdps(rate),
  .calibrated(calibrated),.degraded(degraded),.estimate_valid(valid),
  .x_mm(x),.y_mm(y),.z_mm(z),.vx_mmps(vx),.vy_mmps(vy),.vz_mmps(vz));
 wire xy_valid;
 wire signed[31:0] xx,yy,zz;
 PositionEstimator #(.CLK_FREQ(10000),.SAMPLE_HZ(100),.CALIBRATION_SAMPLES(4),.MAX_TRACK_SECONDS(2),.TRACK_Z(0)) xy_only(
  .clk(clk),.rst(rst),.enable(enable),.sample_valid(sample_valid),.imu_valid(healthy),.accel_clipped(clipped),
  .accel_x(ax),.accel_y(ay),.accel_z(az),.roll_cd(roll),.pitch_cd(pitch),.yaw_rate_cdps(rate),
  .estimate_valid(xy_valid),.x_mm(xx),.y_mm(yy),.z_mm(zz));
 always @(posedge clk)begin
  #2;
  if(!rst && zz!=0)$fatal(1,"XY profile integrated Z");
  if(valid && xy_valid && (xx!=x || yy!=y))$fatal(1,"XY profile changed horizontal solution");
 end
 task sample;
  begin repeat(99) @(negedge clk);sample_valid=1;@(negedge clk);sample_valid=0;end
 endtask
 task zero;
  begin enable=0;ax=0;ay=0;az=16384;roll=0;pitch=0;rate=0;clipped=0;
   sample;enable=1;end
 endtask
 initial begin
  $dumpfile("PositionEstimator.vcd");$dumpvars(0,PositionEstimator_tb);
  repeat(3) @(negedge clk);rst=0;
  repeat(4)sample;
  if(!calibrated || valid) $fatal(1,"Calibration/disabled validity");
  enable=1;repeat(100)sample;
  if(!valid || x!=0 || y!=0 || z!=0) $fatal(1,"Rest/gravity removal");
  zero;ax=1671;repeat(100)sample;
  if(!valid || vx<998 || vx>1001 || x<503 || x>507 || y!=0) $fatal(1,"Acceleration integration %d %d",vx,x);
  zero;roll=1000;ay=2845;az=16135;repeat(100)sample;
  if(!valid || y>10 || y< -10 || z>10 || z< -10) $fatal(1,"Tilted gravity removal %d %d",y,z);
  zero;rate=9000;repeat(100)sample;
  rate=0;ax=1671;repeat(100)sample;
  if(!valid || y> -495 || y< -510 || x>10 || x< -10) $fatal(1,"Yaw reference rotation %d %d",x,y);
  sample;if(valid || !degraded) $fatal(1,"Unaided time horizon");
  zero;sample;clipped=1;sample;
  if(valid || !degraded) $fatal(1,"Clipped acceleration did not invalidate dead reckoning");
  clipped=0;sample;if(valid) $fatal(1,"Lost acceleration auto-restored position validity");
  zero;sample;repeat(250) @(negedge clk);
  if(valid || !degraded) $fatal(1,"Missing samples");
  zero;sample;healthy=0;@(negedge clk);#1;if(valid) $fatal(1,"IMU health gate");
  rst=1;healthy=1;enable=0;clipped=0;ax=100;ay=-50;az=16400;roll=0;pitch=0;rate=0;
  repeat(3) @(negedge clk);rst=0;repeat(4)sample;enable=1;repeat(100)sample;
  if(!valid || x!=0 || y!=0 || z!=0) $fatal(1,"Stationary acceleration bias removal");
  az=18071;repeat(10)sample;
  if(!xy_valid || zz!=0 || z<1)$fatal(1,"XY-only vertical profile");
  $display("PASS: position diagnostic gravity/bias, integration, yaw rotation, time horizon, clipping/stale health and origin reset");$finish;
 end
 initial begin #10000000;$fatal(1,"Position watchdog");end
endmodule
