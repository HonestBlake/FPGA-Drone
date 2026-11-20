`timescale 1ns / 1ps
// Bench-only coherent 8x8 snapshot -> USB UART. Drops new frames while busy.
// Packet: 'L8', frame counter LE32, 64 distances LE16, byte-sum LE16.
module DepthFrameStream #(parameter integer CLK_FREQ=10_000_000)(
    input wire clk, rst, frame_valid,
    input wire [31:0] frame_count,
    output wire [5:0] read_address,
    input wire [15:0] distance_mm,
    output reg [15:0] center_mm,
    output reg center_valid,
    output wire uart_tx
);
    reg [15:0] pixels [0:63];
    reg [31:0] sequence_number;
    reg capturing, sending;
    reg [5:0] capture_index;
    reg [7:0] byte_index;
    reg [15:0] checksum;
    reg [17:0] center_sum;
    reg center_good;
    reg [7:0] data;
    wire ready;
    assign read_address=capture_index;
    always @* begin
        if(byte_index==0) data=8'h4c;
        else if(byte_index==1) data=8'h38;
        else if(byte_index<6) data=sequence_number[(byte_index-2)*8 +: 8];
        else if(byte_index<134)
            data=byte_index[0] ? pixels[(byte_index-6)>>1][15:8] : pixels[(byte_index-6)>>1][7:0];
        else data=byte_index==134 ? checksum[7:0] : checksum[15:8];
    end
    UartOutput #(.CLK_FREQ(CLK_FREQ),.FIFO_DEPTH(16)) output_uart(
        .clk(clk),.rst(rst),.data(data),.valid(sending),.ready(ready),.busy(),.tx(uart_tx));
    always @(posedge clk) begin
        if(rst) begin
            capturing<=0; sending<=0; capture_index<=0; byte_index<=0;
            checksum<=0; sequence_number<=0;
            center_sum<=0; center_good<=1; center_mm<=0; center_valid<=0;
        end else begin
            if(frame_valid && !capturing && !sending) begin
                capturing<=1; capture_index<=0; sequence_number<=frame_count;
                center_sum<=0; center_good<=1;
            end
            if(capturing) begin
                pixels[capture_index]<=distance_mm;
                if(capture_index==27 || capture_index==28 || capture_index==35 || capture_index==36) begin
                    center_sum<=center_sum+distance_mm;
                    if(distance_mm==0 || distance_mm>=4000) center_good<=0;
                end
                if(capture_index==63) begin
                    capturing<=0; sending<=1; byte_index<=0; checksum<=0;
                    center_mm<=(center_sum+18'd2)>>2; center_valid<=center_good;
                end else capture_index<=capture_index+1'b1;
            end
            if(sending && ready) begin
                if(byte_index<134) checksum<=checksum+data;
                if(byte_index==135) sending<=0;
                else byte_index<=byte_index+1'b1;
            end
        end
    end
endmodule
