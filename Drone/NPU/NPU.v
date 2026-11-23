`timescale 1ns / 1ps

`include "layer.sv"
// Display-only inference boundary. ENABLE=0 preserves the production placeholder
// contract; the board test enables inference, never motor overrides.
module NPU #(
    parameter integer ENABLE = 0,
    parameter WEIGHTS_FILE = "weights.mem", BIASES_FILE = "biases.mem"
)(
    input wire clk, rst,
    output wire available, action_valid,
    output wire [2:0] action,
    input wire healthy, pixel_valid,
    input wire [5:0] pixel_address,
    input wire [15:0] pixel_mm
);
    generate if (ENABLE) begin : inference
        reg [15:0] pixels [0:63];
        reg [6:0] loaded;
        reg running, have_prediction, result_pulse;
        reg [2:0] result;
        wire accept_pixel = healthy && !rst && !running && pixel_valid &&
            (pixel_address==0 || (loaded!=0 && {1'b0,pixel_address}==loaded));
        // Single write port, without resetting RAM contents: infer LUT RAM
        // instead of 1024 resettable flip-flops and a large read multiplexer.
        always @(posedge clk) if (accept_pixel) pixels[pixel_address]<=pixel_mm;
        wire [5:0] address;
        wire [2:0] prediction;
        wire done;
        wire signed [31:0] scores [0:4];
        wire done_array [0:4];
        layer #(.N_NEURON(5), .N_INPUTS(64),
            .WEIGHTS_FILE(WEIGHTS_FILE), .BIASES_FILE(BIASES_FILE)) classifier (
            .clk(clk), .rst(rst || !healthy || !running),
            .lidar_x(pixels[address]), .lidar_address(address),
            .results(scores), .done_array(done_array),
            .prediction(prediction), .prediction_valid(done));
        assign available = have_prediction && healthy && !rst;
        assign action_valid = result_pulse && healthy && !rst;
        assign action = result;
        always @(posedge clk) begin
            if (rst || !healthy) begin
                loaded<=0; running<=0; have_prediction<=0; result_pulse<=0; result<=0;
            end else begin
                result_pulse<=0;
                if (running) begin
                    // The private snapshot cannot change during inference.
                    if (done) begin
                        result<=prediction; result_pulse<=1; have_prediction<=1;
                        running<=0; loaded<=0;
                    end
                end else if (pixel_valid) begin
                    if (pixel_address==0) begin
                        loaded<=1;
                    end else if (loaded!=0 && {1'b0,pixel_address}==loaded) begin
                        loaded<=loaded+1'b1;
                        if (pixel_address==63) running<=1;
                    end else loaded<=0;
                end
            end
        end
    end else begin : disabled
        assign available=1'b0;
        assign action_valid=1'b0;
        assign action=3'd0;
    end endgenerate
endmodule
