`include "../../../tests/support/XadcModel.vh"
`include "Icm20948Model.vh"
`timescale 1ns/1ps
module PitchDiagnosticsBench_tb;
 reg clk=0,tx=1,feed_radio=0,btnC=0;
 reg [15:0] sw=16'h0110;
 reg [15:0] roll_us=1500,pitch_us=1500,yaw_us=1500,thrust_us=1000;
 reg [7:0] bytes[0:31];
 wire cs,sclk,mosi,miso;
 wire [6:0] seg;wire[3:0] an;wire dp;
 integer n,checksum;
 reg [43:0] motor_commands;
 always #166.667 clk=~clk;
 BatteryLightsFlightTest #(.CLK_FREQ(3000000),.CORE_CLK_DIV(1),.BAUD_RATE(300000),
  .DISPLAY_REFRESH_HZ(10000),.MOTOR_STARTUP_MS(2),
  .IMU_POWERUP_CYCLES(20),.IMU_RESET_CYCLES(20),.IMU_WAKE_CYCLES(20),.IMU_CALIBRATION_SAMPLES(4)) dut(
  .clk(clk),.btnC(btnC),.tx(tx),.sw(sw),.battery_vin_p(1'b0),.battery_vin_n(1'b0),
  .lidar_rx(1'b1),.ultrasonic_echo(1'b0),.imu_spi_cs_n(cs),.imu_spi_sclk(sclk),
  .imu_spi_mosi(mosi),.imu_spi_miso(miso),.seg(seg),.an(an),.dp(dp));
 Icm20948Model sensor(dut.rst,cs,sclk,mosi,miso);
 task clocks;input integer c;begin repeat(c)@(negedge clk);end endtask
 task send;input[7:0] v;integer b;
  begin tx=0;clocks(10);for(b=0;b<8;b=b+1)begin tx=v[b];clocks(10);end tx=1;clocks(10);end
 endtask
 task frame;
  begin
   bytes[0]=32;bytes[1]=64;
   for(n=1;n<=14;n=n+1)begin bytes[2*n]=8'he8;bytes[2*n+1]=3;end
   bytes[2]=roll_us&255;bytes[3]=roll_us>>8;
   bytes[4]=pitch_us&255;bytes[5]=pitch_us>>8;
   bytes[6]=thrust_us&255;bytes[7]=thrust_us>>8;
   bytes[8]=yaw_us&255;bytes[9]=yaw_us>>8;
   checksum=65535;for(n=0;n<30;n=n+1)checksum=checksum-bytes[n];bytes[30]=checksum&255;bytes[31]=checksum>>8;
   for(n=0;n<32;n=n+1)send(bytes[n]);clocks(30);
  end
 endtask
 function [6:0] glyph;input[3:0] d;
  begin case(d)
   0:glyph=7'h40;1:glyph=7'h79;2:glyph=7'h24;3:glyph=7'h30;4:glyph=7'h19;
   5:glyph=7'h12;6:glyph=7'h02;7:glyph=7'h78;8:glyph=7'h00;9:glyph=7'h10;
   14:glyph=7'h06;default:glyph=7'h7f;
  endcase end
 endfunction
 task view;
  input[15:0] selection,token;
  input[3:0] points;
  integer c,d,seen;reg[6:0] wanted;
  begin
   sw=(sw&16'h800f)|selection;clocks(650);
   if(dut.display_value!==token)$fatal(1,"Pitch view token got %h expected %h",dut.display_value,token);
   seen=0;
   for(c=0;c<350;c=c+1)begin
    if(an!=15)begin
     case(an)14:d=0;13:d=1;11:d=2;7:d=3;default:$fatal(1,"Anode");endcase
     wanted=glyph(token[d*4 +: 4]);
     if(d==3)begin
      if(token[15:12]==4'hc)wanted=7'h7f;
      if(token[15:12]==4'hd)wanted=7'h3f;
     end
     if(d==3 && dut.roll_view_active)wanted=dut.roll_cd[15] ? 7'h3f : 7'h7f;
     if(token==16'hffff)wanted=7'h3f;
     if(seg!==wanted || dp!==!points[d])$fatal(1,"Pitch glyph token %h digit %0d got %h expected %h",token,d,seg,wanted);
     seen=seen|(1<<d);
    end
    @(negedge clk);
   end
   if(seen!=15)$fatal(1,"Incomplete display sweep");
  end
 endtask
 initial begin
  wait(!dut.rst);
  forever begin if(feed_radio)frame;clocks(6000);end
 end
 task frames;input integer count;
  begin repeat(count)@(posedge dut.frame_valid);clocks(30);end
 endtask
 task gyro;input integer raw;
  begin sensor.gy=raw;repeat(3)@(posedge dut.imu_sample_valid);clocks(30);end
 endtask
 initial begin
  // A small stationary mounting offset must remain visible after gyro calibration.
  sensor.ax=500;
  wait(dut.imu_valid);clocks(30);
  if(dut.measured_pitch_cd!=174)$fatal(1,"Calibration unexpectedly zeroed pitch angle");
  view(16'h0110,16'hc017,4'b0010); // SW8 + SW4: +1.7deg, no radio needed
  view(16'h0140,16'hc000,4'b0010); // SW8 + SW6: pitch rate, no radio needed
  view(16'h0120,16'hffff,4'b0000); // Correction requires controller readiness
  view(16'h0010,16'hc000,4'b0010); // SW8 OFF still gives yaw rate
  view(16'h0100,16'h0000,4'b0000); // SW8 alone still gives roll
  sw=15;pitch_us=1505;feed_radio=1;frames(3);
  view(16'h0120,16'hc000,4'b0000);
  sw[15]=1;frames(2);if(!dut.fc_armed)$fatal(1,"Pitch test arming");
  thrust_us=1500;frames(2);clocks(6000);
  if(dut.pitch_setpoint!=0)$fatal(1,"Centered +5 RC pitch bypassed deadband");
  view(16'h0110,16'hc017,4'b0010);
  view(16'h0120,16'hd021,4'b0000); // Nose up -> request nose-down correction
  if(dut.fc_fr!=1026 || dut.fc_fl!=1026 || dut.fc_br!=1068 || dut.fc_bl!=1068)
   $fatal(1,"Nose-down correction does not increase rear/reduce front pair");
  motor_commands={dut.fc_fr,dut.fc_fl,dut.fc_br,dut.fc_bl};
  view(16'h0140,16'hc000,4'b0010);view(16'h0170,16'hc017,4'b0010); // SW4 wins
  view(16'h0160,16'hd021,4'b0000); // SW5 wins over SW6
  if({dut.fc_fr,dut.fc_fl,dut.fc_br,dut.fc_bl}!==motor_commands)$fatal(1,"Pitch display altered motor commands");
  view(16'h1000,16'hc005,4'b0000); // SW12 remains the received pitch stick
  sw[15]=0;clocks(10);
  gyro(-1310);view(16'h0140,16'hc100,4'b0010); // Positive nose-up rate
  gyro(1310);view(16'h0140,16'hd100,4'b0010);
  gyro(-13100);view(16'h0140,16'hc100,4'b0000); // 100deg/s autorange
  gyro(13100);view(16'h0140,16'hd100,4'b0000);
  gyro(0);
  // Recalibrate with the opposite static tilt: same reference, opposite correction.
  thrust_us=1000;pitch_us=1500;sw=15;sensor.ax=-500;btnC=1;clocks(20);btnC=0;
  wait(!dut.rst);wait(dut.imu_valid);clocks(30);frames(3);
  view(16'h0110,16'hd017,4'b0010);
  sw[15]=1;frames(2);thrust_us=1500;frames(2);clocks(6000);
  view(16'h0120,16'hc021,4'b0000); // Nose down -> request nose-up correction
  if(dut.fc_fr!=1068 || dut.fc_fl!=1068 || dut.fc_br!=1026 || dut.fc_bl!=1026)
   $fatal(1,"Nose-up correction does not increase front/reduce rear pair");
  pitch_us=2000;frames(2);clocks(6000);
  if(dut.pitch_setpoint!=-2000)$fatal(1,"Pilot forward pitch sign/scale changed");
  view(16'h0120,16'hd228,4'b0000);
  pitch_us=1000;frames(2);clocks(6000);
  if(dut.pitch_setpoint!=2000)$fatal(1,"Pilot backward pitch sign/scale changed");
  view(16'h0120,16'hc271,4'b0000);
  pitch_us=2000;frames(2);gyro(-30000);frames(2);
  view(16'h0120,16'hd400,4'b0000); // Existing combined correction clamp
  sw[15]=0;clocks(10);view(16'h0120,16'hc000,4'b0000);
  gyro(0);sw[14]=1;clocks(10);
  if(dut.display_value!==dut.error_code)$fatal(1,"Error priority over pitch diagnostic");
  sw[14]=0;feed_radio=0;wait(!dut.rc_valid);
  view(16'h0120,16'hffff,4'b0000);
  view(16'h0140,16'hc000,4'b0010); // Measured rate survives radio loss
  if(!dut.yaw_diag_valid)$fatal(1,"Pitch rate gated by radio readiness");
  sensor.disconnected=1;wait(!dut.imu_valid);
  view(16'h0110,16'hffff,4'b0000);view(16'h0140,16'hffff,4'b0000);
  force dut.configuring=1;#1;
  if(dut.display_value!==16'hc0f6)$fatal(1,"Configuration priority");
  release dut.configuring;
  if(dut.hover_active || dut.controller.npu_available)$fatal(1,"Pitch diagnostics enabled another control mode");
  $display("PASS: SPI/iBUS pitch angle/rate/correction, static calibration reference, motor-pair signs, pilot/deadband/clamp behavior, health, priorities and display isolation");$finish;
 end
 initial begin #1000000000;$fatal(1,"Pitch diagnostics watchdog");end
endmodule
