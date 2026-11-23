`include "../../../tests/support/XadcModel.vh"
`include "Icm20948Model.vh"
`timescale 1ns/1ps
module HoverBench_tb;
 reg clk=0,tx=1,echo=0,feed_echo=1,feed_radio=1;
 reg [15:0] sw=16'h000f;
 integer thrust_us=1000,mode_us=1000,echo_width_ns=3529412;
 wire trigger,cs,sclk,mosi,miso;
 wire [3:0] esc;
 wire [15:0] led;
 reg [7:0] bytes[0:31];
 integer n,checksum,holding,expected;
 always #166.667 clk=~clk;
 BatteryLightsFlightTest #(.CLK_FREQ(3_000_000),.CORE_CLK_DIV(1),.BAUD_RATE(300000),
  .IMU_POWERUP_CYCLES(20),.IMU_RESET_CYCLES(20),.IMU_WAKE_CYCLES(20),
  .IMU_CALIBRATION_SAMPLES(4),.MOTOR_STARTUP_MS(2)) dut(
  .clk(clk),.btnC(1'b0),.tx(tx),.sw(sw),.battery_vin_p(1'b0),.battery_vin_n(1'b0),
  .lidar_rx(1'b1),.ultrasonic_echo(echo),.ultrasonic_trigger(trigger),
  .imu_spi_miso(miso),.imu_spi_cs_n(cs),.imu_spi_sclk(sclk),.imu_spi_mosi(mosi),.esc(esc),.led(led));
 Icm20948Model sensor(dut.rst,cs,sclk,mosi,miso);
 always begin
  @(negedge trigger);
  if(feed_echo && !dut.rst)begin #10000;echo=1;#(echo_width_ns);echo=0;end
 end
 task clocks;input integer c;begin repeat(c)@(negedge clk);end endtask
 task send;input[7:0] v;integer b;
  begin tx=0;clocks(10);for(b=0;b<8;b=b+1)begin tx=v[b];clocks(10);end tx=1;clocks(10);end
 endtask
 initial begin
  wait(!dut.rst);
  forever begin
   if(feed_radio)begin
    bytes[0]=32;bytes[1]=64;
    for(n=1;n<=14;n=n+1)begin bytes[2*n]=8'hdc;bytes[2*n+1]=5;end
    bytes[6]=thrust_us&255;bytes[7]=thrust_us>>8;
    bytes[20]=mode_us&255;bytes[21]=mode_us>>8;
    checksum=65535;for(n=0;n<30;n=n+1)checksum=checksum-bytes[n];
    bytes[30]=checksum&255;bytes[31]=checksum>>8;
    for(n=0;n<32;n=n+1)send(bytes[n]);
   end
   clocks(6000);
  end
 end
 task frames;input integer c;begin repeat(c)@(posedge dut.frame_valid);clocks(30);end endtask
 task range_sample;begin @(posedge dut.ultrasonic_sample_valid);clocks(5);end endtask
 function[15:0] packet;input[10:0] t;reg[11:0] payload;
  begin payload={t,1'b0};packet={payload,payload[11:8]^payload[7:4]^payload[3:0]};end
 endfunction
 task motor_frame;input[10:0] command;
  integer b,c,j;integer highs[0:3];reg[15:0] decoded[0:3];
  begin
   repeat(3)@(posedge dut.motors.send);@(posedge clk);#1;
   for(j=0;j<4;j=j+1)decoded[j]=0;
   for(b=15;b>=0;b=b-1)begin
    for(j=0;j<4;j=j+1)highs[j]=0;
    for(c=0;c<10;c=c+1)begin
     for(j=0;j<4;j=j+1)if(esc[j])highs[j]=highs[j]+1;
     @(posedge clk);#1;
    end
    for(j=0;j<4;j=j+1)begin
     if(highs[j]==8)decoded[j][b]=1;
     else if(highs[j]!=4)$fatal(1,"DShot pulse %0d",highs[j]);
    end
   end
   for(j=0;j<4;j=j+1)if(decoded[j]!==packet(command))$fatal(1,"ESC %0d got %h expected %h",j,decoded[j],packet(command));
  end
 endtask
 initial begin
  wait(dut.imu_valid && dut.ultrasonic_valid && dut.motor_ready);frames(3);
  sw[15]=1;frames(2);if(!dut.fc_armed)$fatal(1,"Arm sequence");motor_frame(0);
  thrust_us=1500;frames(2);if(dut.hover_active || dut.flight_throttle!=500)$fatal(1,"Manual input path");motor_frame(1047);
  mode_us=2000;frames(2);
  if(!dut.hover_active || dut.hover_target_mm<599 || dut.hover_target_mm>601 || dut.flight_throttle!=500 || !led[12])
   $fatal(1,"CH10 physical entry/target/indicator");
  thrust_us=1000;frames(2);
  if(!dut.hover_active || dut.flight_throttle!=500 || !dut.fc_armed)$fatal(1,"Low pilot throttle killed hold");motor_frame(1047);
  mode_us=1000;frames(2);
  if(!dut.hover_pickup_wait || dut.flight_throttle!=500 || !dut.fc_armed)$fatal(1,"Switch-OFF cut motor command");
  force dut.heartbeat=0;#1;if(led[12])$fatal(1,"Pickup blink OFF");
  force dut.heartbeat=1;#1;if(!led[12])$fatal(1,"Pickup blink ON");release dut.heartbeat;
  echo_width_ns=3235294;range_sample;
  if(!dut.hover_pickup_wait || dut.flight_throttle<=500)$fatal(1,"Pending pickup stopped regulating altitude");
  holding=dut.flight_throttle;expected=48+holding*1999/1000;motor_frame(expected);
  thrust_us=1000+holding-1;frames(2);if(!dut.hover_pickup_wait)$fatal(1,"Early pickup");
  thrust_us=1000+holding;frames(2);if(!dut.hover_handover)$fatal(1,"Matching throttle did not start handover");
  thrust_us=1000;frames(2);clocks(3000);
  if(!dut.hover_active || dut.flight_throttle==0 || !dut.fc_armed)$fatal(1,"Handover dropped straight to zero");
  wait(!dut.hover_active);clocks(4000);motor_frame(0);
  // Re-enter, then lose echo while OFF/zero: keep collective, signal failure.
  thrust_us=1500;frames(2);mode_us=2000;frames(2);
  if(!dut.hover_active)$fatal(1,"Second engagement");
  thrust_us=1000;mode_us=1000;frames(2);holding=dut.flight_throttle;feed_echo=0;
  wait(!dut.ultrasonic_valid);clocks(5);
  if(!dut.hover_active || !dut.hover_fault || dut.flight_throttle!=holding || !dut.fc_armed)$fatal(1,"No-echo dropped thrust");
  expected=48+holding*1999/1000;motor_frame(expected);
  // SW15 is still immediate stop, regardless of pending pickup or stick.
  sw[15]=0;clocks(5);
  if(dut.hover_active || dut.fc_armed)$fatal(1,"SW15 failed to disarm hover");motor_frame(0);
  // A fresh OFF/ON request is needed after disarm and after restored sensing.
  feed_echo=1;mode_us=1000;thrust_us=1000;range_sample;frames(3);
  sw[15]=1;frames(2);thrust_us=1500;frames(2);mode_us=2000;frames(2);
  if(!dut.hover_active)$fatal(1,"Recovery engage");thrust_us=1000;mode_us=1000;frames(2);
  feed_radio=0;wait(!dut.rc_valid);clocks(5);
  if(dut.hover_active || dut.fc_armed)$fatal(1,"RC loss retained autonomous throttle");motor_frame(0);
  if(dut.controller.npu_available || dut.controller.avoidance_enable)$fatal(1,"NPU override enabled");
  $display("PASS: physical ultrasonic/CH10 -> height capture -> four ESCs, zero-stick hold/OFF pickup, smooth release, sonar freeze, disarm/RC stop and NPU isolation");$finish;
 end
 initial begin #2000000000;$fatal(1,"Hover board watchdog");end
endmodule
