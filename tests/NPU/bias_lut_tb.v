`include "../../Drone/NPU/bias_lut.sv"
`timescale 1ns/1ps
module bias_lut_tb;
`include "model_constants.vh"
reg[2:0] n=0;wire signed[31:0] b;
bias_lut #(.BIASES_FILE("../../Drone/NPU/biases.mem")) dut(n,b);
initial begin #1;if(b!==BIAS_FIRST)$fatal(1,"signed bias");n=4;#1;if(b!==BIAS_LAST)$fatal(1,"last bias");
 n=7;#1;if(b!==0)$fatal(1,"invalid bias");$display("PASS: trained signed biases and bounds");$finish;end
initial begin $dumpfile("bias_lut.vcd");$dumpvars(0,bias_lut_tb);end
initial begin #100000;$fatal(1,"watchdog");end
endmodule
