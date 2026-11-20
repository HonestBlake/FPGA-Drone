module accu(clk, rst, product, enable, acc);
input clk;
input rst;
input enable;
input signed[31:0] product;
output reg signed [31:0] acc;

always @(posedge clk) begin
    if (rst)
        acc <= 0;
    else if(enable)
        acc <= acc + product;
end
endmodule
