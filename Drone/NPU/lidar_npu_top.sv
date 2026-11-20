`include "layer.sv"
`include "LidarArray.v"
module lidar_npu_top (
    input wire clk,
    input wire rst,

    // Physical LiDAR UART pins
    input wire lidar_rx,
    output wire lidar_tx,

    // Classification result
    output reg [2:0] prediction,
    output reg prediction_valid
);

    // LiDAR-to-layer connections
    wire [5:0] lidar_address;
    wire [15:0] lidar_distance;

    wire frame_valid;
    wire image_valid;

    // Layer outputs
    wire signed [31:0] results [0:4];
    wire done_array [0:4];

    wire [2:0] layer_prediction;
    wire layer_valid;

    // Controller
    localparam WAIT_FRAME = 2'd0;
    localparam RUN_NPU    = 2'd1;
    localparam RESET_NPU  = 2'd2;

    reg [1:0] state;

    wire npu_rst;
    assign npu_rst = rst || (state != RUN_NPU);

    // ====================================
    // LiDAR Interface
    // ====================================

    LidarArray lidar (
        .clk(clk),
        .rst(rst),

        .lidar_rx(lidar_rx),
        .lidar_tx(lidar_tx),

        .read_address(lidar_address),
        .read_distance_mm(lidar_distance),

        .image_valid(image_valid),
        .frame_valid(frame_valid),

        .frame_count(),
        .initialized(),
        .error()
    );

    // ====================================
    // Neural Network
    // ====================================

    layer #(
        .N_NEURON(5),
        .N_INPUTS(64)
    ) npu (
        .clk(clk),
        .rst(npu_rst),

        .lidar_x(lidar_distance),
        .lidar_address(lidar_address),

        .results(results),
        .done_array(done_array),

        .prediction(layer_prediction),
        .prediction_valid(layer_valid)
    );

    // ====================================
    // NPU Controller
    // ====================================

    always @(posedge clk) begin
        if (rst) begin
            state <= WAIT_FRAME;
            prediction <= 0;
            prediction_valid <= 0;
        end
        else begin
            prediction_valid <= 0;

            case (state)

                WAIT_FRAME: begin
                    // Wait for a complete scan
                    if (frame_valid && image_valid)
                        state <= RUN_NPU;
                end

                RUN_NPU: begin
                    // Wait for all five neurons
                    if (layer_valid) begin
                        prediction <= layer_prediction;
                        prediction_valid <= 1;
                        state <= RESET_NPU;
                    end
                end

                RESET_NPU: begin
                    // Reset accumulator and counter
                    // before accepting another scan
                    state <= WAIT_FRAME;
                end

                default:
                    state <= WAIT_FRAME;

            endcase
        end
    end

endmodule
