`include "../../Drone/NPU/argmax.sv"
`timescale 1ns/1ps
module argmax_tb;
reg signed[31:0] scores[0:4];wire[2:0] prediction;wire confident;integer i;
argmax dut(.scores(scores),.prediction(prediction),.confident(confident));
task check;
 input [2:0] expected;input accepted;
 begin #1;if(prediction!==expected || confident!==accepted)
  $fatal(1,"winner/confidence got %0d/%b expected %0d/%b",prediction,confident,expected,accepted);end
endtask
initial begin
 for(i=0;i<5;i=i+1)scores[i]=-100;check(0,0);
 // All five labels, including Continue, face exactly the same thresholds.
 for(i=0;i<5;i=i+1)begin
  scores[i]=100;check(i,0);scores[i]=20000;check(i,1);scores[i]=-100;
 end
 for(i=0;i<5;i=i+1)scores[i]=-100000;
 scores[2]=-1;check(2,0); // A huge lead cannot rescue a below-threshold score.
 for(i=0;i<5;i=i+1)scores[i]=-100;
 scores[2]=4095;scores[1]=-100;check(2,0);
 scores[2]=4096;scores[1]=0;check(2,1); // Inclusive score/margin boundary.
 scores[2]=10000;scores[1]=5905;check(2,0); // Absolute margin 4095.
 scores[1]=5904;check(2,1);
 scores[2]=20000;scores[1]=15001;check(2,0); // Relative margin below 25%.
 scores[1]=15000;check(2,1);
 scores[2]=20003;scores[1]=15003;check(2,0); // Round relative threshold upward.
 scores[1]=15002;check(2,1);
 scores[0]=20003;check(0,0); // Tied top scores never default to Continue.
 scores[0]=21000;scores[2]=20003;check(0,0); // Unique Continue still needs a fair margin.
 for(i=0;i<5;i=i+1)scores[i]=32'sh80000000;
 scores[3]=32'sh7fffffff;check(3,1); // Full signed score range, no overflow.
 if(dut.score_margin!==33'h0ffffffff)$fatal(1,"Score-margin overflow");
 scores[4]=32'sh7ffffffe;check(3,0);
 $display("PASS: five-class confidence fairness, absolute/relative thresholds, rounding, negative/tied scores and full-width margin");$finish;
end
initial begin $dumpfile("argmax.vcd");$dumpvars(0,argmax_tb);end
initial begin #100000;$fatal(1,"watchdog");end
endmodule
