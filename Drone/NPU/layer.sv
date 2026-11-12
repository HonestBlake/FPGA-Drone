
`include "neuron.v"
`include "weight_lut.sv"
`include "bias_lut.sv"
`include "argmax.sv"

module layer #(
    parameter N_NEURON = 5,
    parameter N_INPUTS = 64
)(
    input wire clk,
    input wire rst,

    // Connection to LidarArray
    input wire [15:0] lidar_x,
    output wire [5:0] lidar_address,

    // Neural network outputs
    output wire signed [31:0] results [0:N_NEURON-1],
    output wire done_array [0:N_NEURON-1],
    output wire [2:0] prediction,
    output wire prediction_valid
);

    reg [7:0] counter;

    wire signed [15:0] w [0:N_NEURON-1];
    wire signed [31:0] b [0:N_NEURON-1];

    wire [15:0] x;

    // Request a pixel from the LiDAR buffer
    assign lidar_address = counter[5:0];

    // Send the selected distance to all neurons
    assign x = lidar_x;

    genvar i;

    generate
        for (i = 0; i < N_NEURON; i = i + 1) begin : neuron_array

            wire [7:0] neuron_number;
            assign neuron_number = i;

            weight_lut #(
                .N_INPUTS(N_INPUTS),
                .N_NEURON(N_NEURON)
            ) weight (
                .address(counter),
                .n_number(neuron_number),
                .weight(w[i])
            );

            bias_lut biases (
                .neuron_addr(neuron_number[2:0]),
                .bias(b[i])
            );

            neuron #(
                .N_INPUTS(N_INPUTS)
            ) n (
                .clk(clk),
                .rst(rst),
                .counter(counter),
                .x(x),
                .w(w[i]),
                .bias(b[i]),
                .result(results[i]),
                .done(done_array[i])
            );

        end
    endgenerate

    // Counter for the 64 LiDAR measurements
    always @(posedge clk) begin
        if (rst)
            counter <= 0;
        else if (counter < N_INPUTS)
            counter <= counter + 1;
    end

    // Select the neuron with the highest score
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
