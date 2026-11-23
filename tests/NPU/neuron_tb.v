`include "../../Drone/NPU/neuron.v"
`timescale 1ns/1ps
module neuron_tb;
reg clk=0,rst=1;reg[7:0] count=0;reg[15:0] x=0;wire signed[31:0] result;wire done;
always #5 clk=~clk;neuron #(.N_INPUTS(3)) dut(clk,rst,count,x,-16'sd2,32'sd10,result,done);
initial begin repeat(2)@(negedge clk);rst=0;x=3;
 @(negedge clk);count=1;x=4;if(done)$fatal(1,"early done");
 @(negedge clk);count=2;x=8;@(negedge clk);count=3;
 @(negedge clk);if(!done || result!==-20)$fatal(1,"last pixel/bias");
 repeat(3)@(negedge clk);if(result!==-20)$fatal(1,"result changed");
 rst=1;@(negedge clk);if(done || result!==0)$fatal(1,"reset");$display("PASS: neuron final input, signed bias, completion and reset");$finish;end
initial begin $dumpfile("neuron.vcd");$dumpvars(0,neuron_tb);end
initial begin #100000;$fatal(1,"watchdog");end
endmodule
