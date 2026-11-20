
module weight_lut #(
    parameter N_INPUTS = 64,
    parameter N_NEURON = 5
)(
    input [7:0] address,
    input [7:0] n_number,
    output logic signed [15:0] weight
);

logic signed [15:0] weights [0:N_INPUTS*N_NEURON-1];

initial begin
    $readmemh("weights.mem", weights);
end

always_comb begin
    if (n_number < N_NEURON && address < N_INPUTS)
        weight = weights[n_number*N_INPUTS + address];
    else
        weight = 16'sd0;
end

endmodule
