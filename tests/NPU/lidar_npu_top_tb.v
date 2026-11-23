`include "../../Drone/PeripheralInterface/UartReceiver.v"
`include "../../Drone/UartTransmitter.v"
`include "../../Drone/NPU/lidar_npu_top.sv"
`timescale 1ns/1ps
module lidar_npu_top_tb;
`include "model_constants.vh"
reg clk=0,rst=1;wire[2:0] prediction;wire valid;
wire [15:0] rejected_pixel=REJECTED_FRAME_PIXELS[dut.lidar_address*16 +:16];
always #5 clk=~clk;lidar_npu_top dut(.clk(clk),.rst(rst),.lidar_rx(1'b1),.prediction(prediction),.prediction_valid(valid));
defparam dut.npu.WEIGHTS_FILE="../../Drone/NPU/weights.mem";
defparam dut.npu.BIASES_FILE="../../Drone/NPU/biases.mem";
initial begin repeat(3)@(negedge clk);rst=0;
 force dut.lidar_distance=16'd1000;force dut.frame_valid=1;force dut.image_valid=1;
 @(negedge clk);release dut.frame_valid;release dut.image_valid;
 wait(dut.state==2);#1;
 if(valid!==UNIFORM_1000_CONFIDENT || prediction!==(UNIFORM_1000_CONFIDENT ? UNIFORM_1000_CLASS : 3'd7))
  $fatal(1,"reference top prediction/confidence");
 repeat(4)@(negedge clk);if(valid)$fatal(1,"reference top pulse");
 wait(dut.state==0);
 force dut.lidar_distance=rejected_pixel;force dut.frame_valid=1;force dut.image_valid=1;
 @(negedge clk);release dut.frame_valid;release dut.image_valid;
 wait(dut.state==2);#1;
 if(valid || prediction!==7)$fatal(1,"reference top ambiguous frame retained a command");
 release dut.lidar_distance;$display("PASS: reference top accepted/rejected inference sequencing and no default Continue");$finish;end
initial begin $dumpfile("lidar_npu_top.vcd");$dumpvars(0,lidar_npu_top_tb);end
initial begin #100000;$fatal(1,"watchdog");end
endmodule
