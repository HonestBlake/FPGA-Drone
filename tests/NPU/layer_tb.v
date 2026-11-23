`include "../../Drone/NPU/layer.sv"
`timescale 1ns/1ps
module layer_tb;
reg clk=0,rst=1;wire[5:0] address;wire signed[31:0] scores[0:4];wire done_array[0:4];wire[2:0] prediction;wire valid;
always #5 clk=~clk;
layer #(.WEIGHTS_FILE("../../Drone/NPU/weights.mem"),.BIASES_FILE("../../Drone/NPU/biases.mem")) dut(
 .clk(clk),.rst(rst),.lidar_x(address==63 ? 16'd2000 : 16'd0),.lidar_address(address),
 .results(scores),.done_array(done_array),.prediction(prediction),.prediction_valid(valid));
initial begin repeat(3)@(negedge clk);rst=0;wait(valid);#1;
 if(scores[0]!==-15556 || scores[1]!==-20231 || scores[2]!==6647 || scores[3]!==-6227 || scores[4]!==35367 || prediction!==4)
 $fatal(1,"layer last pixel, biases or argmax");
 rst=1;repeat(2)@(negedge clk);if(valid)$fatal(1,"layer reset");$display("PASS: layer all-neuron scores, final pixel and reset");$finish;end
initial begin $dumpfile("layer.vcd");$dumpvars(0,layer_tb);end
initial begin #100000;$fatal(1,"watchdog");end
endmodule
