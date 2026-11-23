`include "../../../tests/support/XadcModel.vh"
`timescale 1ns/1ps
module LidarBench_tb;
 reg clk=0,btn=1,rx=1;
 wire ltx,usb;
 wire [15:0] led; reg [15:0] sw=16'h0200;
 reg [7:0] packet[0:135];
 integer n,b,sum;
 always #41.667 clk=~clk;
 BatteryLightsFlightTest #(.CLK_FREQ(12_000_000),.CORE_CLK_DIV(1),
  .LIDAR_STARTUP_MS(1),.LIDAR_SETTLE_MS(1),.LIDAR_POLL_MS(1),
  .LIDAR_RESPONSE_TIMEOUT_MS(30),.LIDAR_BYTE_TIMEOUT_MS(1),.LIDAR_FRESH_MS(20)) dut(
  .clk(clk),.btnC(btn),.tx(1'b1),.sw(sw),.battery_vin_p(1'b0),.battery_vin_n(1'b0),
  .imu_spi_miso(1'b0),.ultrasonic_echo(1'b0), .ultrasonic_trigger(), .lidar_rx(rx),.lidar_tx(ltx),.usb_uart_tx(usb),.led(led));
 task send;
  input [7:0] value;
  integer bitno;
  begin rx=0; #8681;
   for(bitno=0;bitno<8;bitno=bitno+1) begin rx=value[bitno]; #8681; end
   rx=1; #8681;
  end
 endtask
 task receive;
  output [7:0] value;
  integer bitno;
  begin @(negedge usb); #13000;
   for(bitno=0;bitno<8;bitno=bitno+1) begin value[bitno]=usb; #8667; end
   if(usb!==1) $fatal(1,"USB stop");
  end
 endtask
 initial begin #200_000_000; $fatal(1,"Lidar top watchdog"); end
 initial begin
  repeat(300) @(negedge clk); btn=0;
  wait(dut.lidar.state==4); send(8'h53);send(1);send(0);send(0);
  wait(dut.lidar.state!=4); wait(dut.lidar.state==4);
  fork
   begin
    send(8'h53);send(2);send(128);send(0);
    for(n=0;n<64;n=n+1) begin send((1200+n)&255);send((1200+n)>>8);end
   end
   begin
    for(b=0;b<136;b=b+1) receive(packet[b]);
   end
  join
  if(dut.lidar_center_mm!=1232 || !dut.lidar_center_valid) $fatal(1,"Top centre average");
  force dut.configuring=0; #1;
  if(!dut.npu_available || dut.npu_action!==4 || dut.display_value!==16'hb101)
   $fatal(1,"SW9 real-weight NPU Stop for ramp frame");
  force dut.ai_override_request=1;
  for(n=0;n<5;n=n+1) begin
   force dut.npu_action=n; #1;
   case(n)
    0: if(dut.display_value!==16'hb100)$fatal(1,"Continue mapping");
    1: if(dut.display_value!==16'hb103)$fatal(1,"Left mapping");
    2: if(dut.display_value!==16'hb104)$fatal(1,"Right mapping");
    3: if(dut.display_value!==16'hb102)$fatal(1,"Up mapping");
    4: if(dut.display_value!==16'hb101)$fatal(1,"Stop mapping");
   endcase
   if(dut.controller.npu_available!==0 || dut.controller.avoidance_enable!==0)
    $fatal(1,"NPU connected to motor override");
  end
  release dut.npu_action;release dut.ai_override_request;
  release dut.configuring;
  if(!dut.lidar_healthy || led[9]!==1 || dut.lidar_frame_count!=1)
   $fatal(1,"Complete frame did not make lidar healthy");
  if(led[15]!==dut.heartbeat || dut.fc_armed) $fatal(1,"Lidar armed flight controller");
  for(n=0;n<64;n=n+1)
   if({packet[7+2*n],packet[6+2*n]}!==1200+n) $fatal(1,"Top pixel routing %0d",n);
  sum=0;for(n=0;n<134;n=n+1) sum=(sum+packet[n])&65535;
  if({packet[135],packet[134]}!==sum[15:0]) $fatal(1,"Top USB checksum");
  wait(!dut.lidar_healthy);
  force dut.configuring=0;#1;
  if(dut.npu_available || dut.display_value!==16'hffff)$fatal(1,"Stale NPU result displayed");
  release dut.configuring;
  force dut.heartbeat=0; #1;if(led[9]!==0) $fatal(1,"Stale lidar blink off");
  force dut.heartbeat=1; #1;if(led[9]!==1) $fatal(1,"Stale lidar blink on");
  release dut.heartbeat;
  $display("PASS: sensor UART -> complete 8x8 buffer -> coherent USB packet, lidar/NPU health/freshness, five labels and isolated flight control");
  $finish;
 end
endmodule
