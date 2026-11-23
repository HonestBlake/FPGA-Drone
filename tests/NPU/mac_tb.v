`include "../../Drone/NPU/mac.v"
`timescale 1ns/1ps
module mac_tb;
reg clk=0,rst=1,en=0;reg[15:0] a=65535;reg signed[15:0] b=-2;wire signed[31:0] acc;
always #5 clk=~clk;mac dut(clk,rst,a,b,en,acc);
initial begin repeat(2)@(negedge clk);rst=0;en=1;
 repeat(2)@(negedge clk);if(acc!==-262140)$fatal(1,"MAC signed result");
 en=0;@(negedge clk);if(acc!==-262140)$fatal(1,"MAC disabled");
 rst=1;@(negedge clk);if(acc!==0)$fatal(1,"MAC reset");$display("PASS: MAC arithmetic, enable and reset");$finish;end
initial begin $dumpfile("mac.vcd");$dumpvars(0,mac_tb);end
initial begin #100000;$fatal(1,"watchdog");end
endmodule
