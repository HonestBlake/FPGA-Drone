`timescale 1ns/1ps
module DepthFrameStream_tb;
 reg clk=0,rst=1,frame_valid=0;
 reg [31:0] frame_count=32'h12345678;
 wire [5:0] address;
 reg [15:0] base_value=1000;
 wire [15:0] distance=base_value+address;
 wire tx; wire [15:0] center; wire valid;
 reg [7:0] packet[0:135];
 integer n,b,sum;
 always #125 clk=~clk;
 DepthFrameStream #(.CLK_FREQ(4_000_000)) dut(clk,rst,frame_valid,frame_count,address,distance,center,valid,tx);
 task receive;
  output [7:0] value;
  integer bitno;
  begin
   @(negedge tx); #(35*250+35*125);
   for(bitno=0;bitno<8;bitno=bitno+1) begin value[bitno]=tx; #(35*250); end
   if(tx!==1) $fatal(1,"USB UART stop bit");
  end
 endtask
 initial begin #100_000_000; $fatal(1,"Stream watchdog"); end
 initial begin
  repeat(5) @(negedge clk); rst=0;
  @(negedge clk); frame_valid=1; @(negedge clk); frame_valid=0;
  fork
   begin
    wait(dut.sending); repeat(2) @(negedge clk);
    base_value=2000; frame_count=99; frame_valid=1;
    @(negedge clk); frame_valid=0;
   end
   begin
    for(n=0;n<136;n=n+1) receive(packet[n]);
   end
  join
  if({packet[0],packet[1]}!==16'h4c38 ||
     {packet[5],packet[4],packet[3],packet[2]}!==32'h12345678)
    $fatal(1,"USB frame header/sequence");
  if(center!=1032 || !valid) $fatal(1,"Centre four rounded average");
  sum=0;
  for(n=0;n<134;n=n+1) sum=(sum+packet[n]) & 65535;
  if({packet[135],packet[134]}!==sum[15:0]) $fatal(1,"Checksum");
  for(n=0;n<64;n=n+1)
   if({packet[7+n*2],packet[6+n*2]}!==1000+n) $fatal(1,"Pixel order/coherence %0d",n);
  repeat(400) @(negedge clk);
  if(tx!==1 || dut.sending) $fatal(1,"Dropped frame repeated");
  base_value=4000; frame_valid=1; @(negedge clk); frame_valid=0;
  wait(dut.sending); #1;
  if(valid) $fatal(1,"Invalid centre pixels must not yield valid distance");
  rst=1; repeat(2) @(negedge clk);
  if(tx!==1 || dut.sending || dut.capturing) $fatal(1,"Stream reset");
  $display("PASS: coherent USB depth snapshot, 64 pixels, LE frame ID, checksum, UART timing, busy frame drop, reset");
  $finish;
 end
endmodule
