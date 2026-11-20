`include "../../../tests/support/XadcModel.vh"
`timescale 1ns/1ps
module BoardClock_tb;
 reg clk=0,tx=1;
 wire trig;
 wire[3:0] esc;
 reg[7:0] bytes[0:31];
 integer n,b,sum;
 time began;
 always #5 clk=~clk;
 BatteryLightsFlightTest dut(.clk(clk),.btnC(1'b0),.tx(tx),.sw(16'b0),
  .battery_vin_p(1'b0),.battery_vin_n(1'b0),.imu_spi_miso(1'b0),.lidar_rx(1'b1),
  .ultrasonic_echo(1'b0),.ultrasonic_trigger(trig),.esc(esc));
 task send;
  input[7:0] value;integer bitno;
  begin tx=0;#8681;for(bitno=0;bitno<8;bitno=bitno+1)begin tx=value[bitno];#8681;end tx=1;#8681;end
 endtask
 initial begin #10000000;$fatal(1,"Board clock watchdog");end
 initial begin
  @(posedge dut.logic_clk);began=$time;@(posedge dut.logic_clk);
  if($time-began!=200) $fatal(1,"5MHz core period");
  @(posedge trig);began=$time;@(negedge trig);
  if($time-began!=20000)$fatal(1,"Real board trigger pulse width");
  @(posedge dut.motors.send);@(posedge esc[0]);began=$time;@(negedge esc[0]);
  if($time-began!=1200)$fatal(1,"5MHz DShot300 zero pulse");
  bytes[0]=32;bytes[1]=64;
  for(n=1;n<=14;n=n+1)begin bytes[2*n]=8'hdc;bytes[2*n+1]=5;end
  bytes[6]=8'he8;bytes[7]=3; // throttle 1000
  sum=65535;for(n=0;n<30;n=n+1)sum=sum-bytes[n];bytes[30]=sum&255;bytes[31]=sum>>8;
  for(n=0;n<32;n=n+1)send(bytes[n]);
  #20000;if(!dut.rc_valid || dut.throttle!=0 || dut.fc_armed)$fatal(1,"115200 UART at real 5MHz core");
  $display("PASS: board 100MHz/20 clock, 20us sonar trigger, DShot300 pulse quantization, real 115200 UART and disarmed gating");$finish;
 end
endmodule
