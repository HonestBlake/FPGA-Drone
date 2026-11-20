`include "multiply.v"
`include "accumulator.v"

module mac(clk, rst, a, b, enable, acc);
input clk, rst, enable;
input [15:0] a; 
input signed [15:0] b;
output signed [31:0] acc;

wire signed [31:0] product;


multiply uut1(.a(a), .b(b), .product(product));
accu uut2(.clk(clk), .rst(rst), .product(product), .enable(enable), .acc(acc));

endmodule