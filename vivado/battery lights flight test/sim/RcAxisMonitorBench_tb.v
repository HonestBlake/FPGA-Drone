`include "../../../tests/support/XadcModel.vh"
`include "Icm20948Model.vh"
`timescale 1ns/1ps
module RcAxisMonitorBench_tb;
 reg clk=0,tx=1;
 reg [15:0] sw=16'h000f;
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
  .clk(clk),.btnC(1'b0),.tx(tx),.sw(sw),.battery_vin_p(1'b0),.battery_vin_n(1'b0),
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
  integer c,d,seen;reg[6:0] wanted;
  begin
   sw=(sw&16'h800f)|selection;clocks(650);
   if(dut.display_value!==token)$fatal(1,"RC view token got %h expected %h",dut.display_value,token);
   seen=0;
   for(c=0;c<350;c=c+1)begin
    if(an!=15)begin
     case(an)14:d=0;13:d=1;11:d=2;7:d=3;default:$fatal(1,"Anode");endcase
     wanted=glyph(token[d*4 +: 4]);
     if(d==3)begin
      if(token[15:12]==4'hc)wanted=7'h7f;
      if(token[15:12]==4'hd)wanted=7'h3f;
     end
     if(token==16'hffff)wanted=7'h3f;
     if(seg!==wanted || dp!==1)$fatal(1,"RC glyph token %h digit %0d got %h expected %h",token,d,seg,wanted);
     seen=seen|(1<<d);
    end
    @(negedge clk);
   end
   if(seen!=15)$fatal(1,"Incomplete display sweep");
  end
 endtask
 initial begin
  wait(dut.imu_valid && dut.motor_ready);frame;
  view(16'h0080,16'hc000);view(16'h0800,16'hc000);view(16'h1000,16'hc000);
  // Independent CH1/CH2/CH4 values prove selector routing and sign.
  roll_us=1537;pitch_us=1438;yaw_us=1623;frame;
  view(16'h0080,16'hc123);view(16'h0800,16'hc037);view(16'h1000,16'hd062);
  view(16'h1880,16'hc037); // roll wins if all RC view switches selected
  roll_us=1000;pitch_us=2000;yaw_us=1000;frame;
  view(16'h0080,16'hd500);view(16'h0800,16'hd500);view(16'h1000,16'hc500);
  roll_us=2100;pitch_us=900;yaw_us=2000;frame;
  view(16'h0080,16'hc500);view(16'h0800,16'hc500);view(16'h1000,16'hd500);
  roll_us=1499;pitch_us=1501;yaw_us=1499;frame;
  view(16'h0080,16'hd001);view(16'h0800,16'hd001);view(16'h1000,16'hc001);
  // A view selection cannot change motor commands, even with the controller armed.
  roll_us=1500;pitch_us=1500;yaw_us=1500;sw=15;frame;clocks(6000);
  sw[15]=1;frame;if(!dut.fc_armed)$fatal(1,"Arming for observer-isolation test");
  thrust_us=1500;frame;clocks(6000);
  motor_commands={dut.fc_fr,dut.fc_fl,dut.fc_br,dut.fc_bl};
  view(16'h0080,16'hc000);view(16'h0800,16'hc000);view(16'h1000,16'hc000);
  if({dut.fc_fr,dut.fc_fl,dut.fc_br,dut.fc_bl}!==motor_commands || !dut.fc_armed || dut.hover_active)
   $fatal(1,"Display changed flight commands/arming/hover");
  sw[15]=0;
  // Existing error/config precedence and invalid-radio dashes remain intact.
  sw[14]=1;clocks(10);if(dut.display_value!==dut.error_code)$fatal(1,"Error priority");
  sw[14]=0;wait(!dut.rc_valid);view(16'h0080,16'hffff);view(16'h0800,16'hffff);view(16'h1000,16'hffff);
  force dut.configuring=1;#1;if(dut.display_value!==16'hc0f6)$fatal(1,"Config priority");release dut.configuring;
  $display("PASS: physical iBUS -> SW7 yaw/SW11 roll/SW12 pitch signed display, endpoints/neutral/near-zero, priorities, timeout and flight isolation");$finish;
 end
 initial begin #500000000;$fatal(1,"RC monitor watchdog");end
endmodule
