`include "neuron.v"
`include "weight_lut.sv"
`include "input_buffer.sv"
`include "bias_lut.sv"
`include "argmax.sv"

module layer#(parameter N_NEURON = 5, parameter N_INPUTS = 64)(
input clk,
input rst,
output signed [31:0] results [0:N_NEURON-1],
output done_array [0:N_NEURON-1],

output wire [2:0] prediction,
output wire prediction_valid);

reg [7:0] counter;
wire signed [15:0] w [0:N_NEURON-1];
wire signed [31:0] b [0:N_NEURON-1];
wire [15:0] x;


input_buffer inputs(.address(counter), .x(x));

genvar i;


generate
    for (i =0; i < N_NEURON; i=i+1) begin : neuron_array

        wire [7:0] neuron_number;
        assign neuron_number = i;
        weight_lut #(.N_INPUTS(N_INPUTS), .N_NEURON(N_NEURON))weight (.address(counter), .n_number(neuron_number), .weight(w[i]));
        bias_lut biases(.neuron_addr(neuron_number[2:0]), .bias(b[i]));
        neuron #(.N_INPUTS(N_INPUTS)) n (.clk(clk), 
        .rst(rst),
        .counter(counter),
         .x(x), 
         .w(w[i]), 
         .bias(b[i]),
         .result(results[i]), 
         .done(done_array[i]));
    end
endgenerate

always @(posedge clk) begin
    if(rst) begin
        counter <= 0;
    end else if (counter < N_INPUTS)begin
        counter <= counter + 1;
    end
end

 argmax classifier (
        .scores(results),
        .prediction(prediction)
    );

assign prediction_valid = &{
        done_array[0],
        done_array[1],
        done_array[2],
        done_array[3],
        done_array[4]
    };
endmodule