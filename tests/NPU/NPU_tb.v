`include "../../Drone/NPU/NPU.v"
`timescale 1ns / 1ps
module NPU_tb;
    reg clk = 0, rst = 1;
    wire available;
    wire action_valid;
    wire [2:0] action;
    integer cycle;
    always #5 clk = ~clk;
    NPU dut (.clk(clk), .rst(rst), .available(available), .action_valid(action_valid), .action(action));
    initial begin
        $dumpfile("NPU.vcd"); $dumpvars(0, NPU_tb);
        for (cycle = 0; cycle < 20; cycle = cycle + 1) begin
            @(negedge clk);
            if (available !== 0) $fatal(1, "NPU placeholder reported available");
            if ({action_valid, action} !== 4'd0) $fatal(1, "NPU placeholder emitted an action");
            rst = (cycle % 5 == 0);
        end
        $display("PASS: NPU placeholder remains unavailable (no inference implemented)");
        $finish;
    end
endmodule
