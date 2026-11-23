`timescale 1ns / 1ps
`ifndef DRONE_NPU_ARGMAX_SV
`define DRONE_NPU_ARGMAX_SV

module argmax #(
    // Heuristic thresholds in raw integer-score units, not probabilities.
    parameter integer MIN_SCORE = 4096,
    parameter integer MIN_MARGIN = 4096,
    // 2 requires a lead of at least ceil(abs(winner)/4), i.e. 25%.
    parameter integer RELATIVE_MARGIN_SHIFT = 2
) (
    input wire signed [31:0] scores [0:4],
    output reg [2:0] prediction,
    output wire confident
);

    integer i;
    reg signed [31:0] max_score;
    reg signed [31:0] second_score;
    // Extend before subtracting: INT_MAX - INT_MIN exceeds signed 32 bits.
    wire [32:0] score_margin = $signed({max_score[31],max_score}) -
                              $signed({second_score[31],second_score});
    wire [31:0] winner_magnitude = max_score[31] ? -max_score : max_score;
    localparam [32:0] RELATIVE_ROUND = (33'd1 << RELATIVE_MARGIN_SHIFT)-1;
    wire [32:0] relative_margin = ({1'b0,winner_magnitude}+RELATIVE_ROUND) >> RELATIVE_MARGIN_SHIFT;
    assign confident = max_score > second_score && max_score >= MIN_SCORE &&
                       score_margin >= MIN_MARGIN && score_margin >= relative_margin;

    always @(*) begin
        prediction = 3'd0;
        max_score = scores[0];
        second_score = 32'sh80000000;

        for (i = 1; i < 5; i = i + 1) begin
            if (scores[i] > max_score) begin
                second_score = max_score;
                max_score = scores[i];
                prediction = i;
            end else if (scores[i] > second_score) second_score = scores[i];
        end
    end

    // synthesis translate_off
    initial begin
        if (MIN_MARGIN < 0 || RELATIVE_MARGIN_SHIFT < 0 || RELATIVE_MARGIN_SHIFT > 16)
            $fatal(1,"argmax: invalid confidence thresholds");
    end
    // synthesis translate_on

endmodule

`endif
