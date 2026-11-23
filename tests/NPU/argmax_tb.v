`include "../../Drone/NPU/argmax.sv"
`timescale 1ns/1ps
module argmax_tb;
reg signed[31:0] scores[0:4];wire[2:0] prediction;integer i;
argmax dut(scores,prediction);
initial begin for(i=0;i<5;i=i+1)scores[i]=-100;#1;if(prediction!==0)$fatal(1,"tie rule");
 for(i=0;i<5;i=i+1)begin scores[i]=100;#1;if(prediction!==i)$fatal(1,"class %0d",i);scores[i]=-100;end
 scores[2]=-1;#1;if(prediction!==2)$fatal(1,"signed compare");$display("PASS: argmax five classes, negative scores and ties");$finish;end
initial begin $dumpfile("argmax.vcd");$dumpvars(0,argmax_tb);end
initial begin #100000;$fatal(1,"watchdog");end
endmodule
