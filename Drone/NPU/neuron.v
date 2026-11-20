`include "mac.v"

module neuron#(parameter N_INPUTS = 5)(clk, rst, counter, x, w, bias, result, done);
input clk, rst;
input [7:0] counter;
input [15:0] x;
input signed [15:0] w;
input signed [31:0] bias;

output reg signed [31:0] result;
output reg done;


wire enable;
wire signed [31:0] acc;

assign enable = (counter < N_INPUTS);
mac uut(.clk(clk), .rst(rst), .a(x), .b(w), .enable(enable), .acc(acc));

always @(posedge clk) begin
    if(rst) begin
        result <= 0;
        done <= 0;
    end
    else
        if(counter == N_INPUTS) begin
            done <= 1;
            result <= acc + bias;
        end

end
endmodule