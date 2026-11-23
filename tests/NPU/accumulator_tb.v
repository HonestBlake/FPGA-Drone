`include "../../Drone/NPU/accumulator.v"
`timescale 1ns/1ps
module accumulator_tb;
reg clk=0,rst=1,en=0;reg signed[31:0] p=0;wire signed[31:0] acc;
always #5 clk=~clk;accumulator dut(clk,rst,p,en,acc);
initial begin repeat(2)@(negedge clk);rst=0;en=1;p=-100;
 repeat(3)@(negedge clk);if(acc!==-300)$fatal(1,"signed accumulation");
 en=0;p=999;repeat(2)@(negedge clk);if(acc!==-300)$fatal(1,"enable");
 rst=1;@(negedge clk);if(acc!==0)$fatal(1,"reset");$display("PASS: accumulator signed sums, enable and reset");$finish;end
initial begin $dumpfile("accumulator.vcd");$dumpvars(0,accumulator_tb);end
initial begin #100000;$fatal(1,"watchdog");end
endmodule
