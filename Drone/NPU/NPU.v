`timescale 1ns / 1ps

// Reserved boundary for the entire NPU. No inference engine exists yet.
// Add implementation modules beneath this folder when its contract is defined.
module NPU (
    input wire clk,
    input wire rst,
    output wire available,
    output wire action_valid,
    output wire [2:0] action // 0 CONTINUE, 1 STOP, 2 UP, 3 LEFT, 4 RIGHT.
);
    assign available = 1'b0;
    assign action_valid = 1'b0;
    assign action = 3'd0;
endmodule
