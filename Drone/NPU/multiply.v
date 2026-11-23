`timescale 1ns / 1ps
`ifndef DRONE_NPU_MULTIPLY_V
`define DRONE_NPU_MULTIPLY_V
module multiply(a, b, product);
input [15:0] a;
input signed [15:0] b;
output signed [31:0] product;

assign product = $signed({1'b0, a}) * b;

endmodule
`endif
