`timescale 1ns / 1ps
`ifndef DRONE_NPU_ARGMAX_SV
`define DRONE_NPU_ARGMAX_SV

module argmax (
    input wire signed [31:0] scores [0:4],
    output reg [2:0] prediction
);

    integer i;
    reg signed [31:0] max_score;

    always @(*) begin
        prediction = 3'd0;
        max_score = scores[0];

        for (i = 1; i < 5; i = i + 1) begin
            if (scores[i] > max_score) begin
                max_score = scores[i];
                prediction = i;
            end
        end
    end

endmodule

`endif
