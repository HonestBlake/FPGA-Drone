`include "../../Drone/NPU/weight_lut.sv"
`timescale 1ns/1ps
module weight_lut_tb;
`include "model_constants.vh"
reg[7:0] address=27,n=0;wire signed[15:0] w;
weight_lut #(.WEIGHTS_FILE("../../Drone/NPU/weights.mem")) dut(address,n,w);
initial begin #1;if(w!==WEIGHT_ROW0_PIXEL27)$fatal(1,"signed weight row0");n=1;#1;if(w!==WEIGHT_ROW1_PIXEL27)$fatal(1,"weight neuron stride");
 address=64;#1;if(w!==0)$fatal(1,"invalid pixel");address=0;n=5;#1;if(w!==0)$fatal(1,"invalid neuron");$display("PASS: trained weight addressing and bounds");$finish;end
initial begin $dumpfile("weight_lut.vcd");$dumpvars(0,weight_lut_tb);end
initial begin #100000;$fatal(1,"watchdog");end
endmodule
