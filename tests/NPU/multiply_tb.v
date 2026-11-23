`include "../../Drone/NPU/multiply.v"
`timescale 1ns/1ps
module multiply_tb;
reg [15:0] a;reg signed[15:0] b;wire signed[31:0] p;
multiply dut(a,b,p);
initial begin
 a=65535;b=-32768;#1;if(p!==-32'sd2147450880)$fatal(1,"unsigned input signed weight");
 b=32767;#1;if(p!==32'sd2147385345)$fatal(1,"positive extreme");
 a=0;b=-12;#1;if(p!==0)$fatal(1,"zero product");
 $display("PASS: multiply unsigned-distance/signed-weight extremes");$finish;end
initial begin $dumpfile("multiply.vcd");$dumpvars(0,multiply_tb);end
initial begin #100000;$fatal(1,"watchdog");end
endmodule
