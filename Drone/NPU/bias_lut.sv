
module bias_lut (
    input  wire [2:0] neuron_addr,
    output wire signed [31:0] bias
);

    // Five 32-bit signed biases
    reg signed [31:0] bias_mem [0:4];

    initial begin
        $readmemh("biases.mem", bias_mem);
    end

    // Select the bias for the requested neuron
    assign bias = (neuron_addr < 5)
                ? bias_mem[neuron_addr]
                : 32'sd0;

endmodule
