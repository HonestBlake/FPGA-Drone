`timescale 1ns / 1ps

// System timing primitive: a one-clk-cycle enable every CLK_FREQ / TICK_FREQ
// cycles. Keep downstream logic on clk and update it with `if (tick)`.
// Frequencies are in Hz; TICK_FREQ must divide CLK_FREQ exactly.
module TickGenerator #(
    parameter integer CLK_FREQ  = 100_000_000, // Basys3 onboard clock
    parameter integer TICK_FREQ = 1_000
) (
    input  wire clk,
    input  wire rst,  // Synchronous, active-high reset; assert before use.
    output reg  tick
);

    // Guard calculations so invalid parameters can be reported in simulation.
    localparam integer CYCLES_PER_TICK =
        (TICK_FREQ > 0 && CLK_FREQ >= TICK_FREQ) ? CLK_FREQ / TICK_FREQ : 1;
    localparam integer COUNTER_WIDTH =
        (CYCLES_PER_TICK > 1) ? $clog2(CYCLES_PER_TICK) : 1;
    localparam [COUNTER_WIDTH-1:0] LAST_COUNT = CYCLES_PER_TICK - 1;

    reg [COUNTER_WIDTH-1:0] counter;

    // synthesis translate_off
    initial begin
        if (CLK_FREQ <= 0 || TICK_FREQ <= 0 || TICK_FREQ > CLK_FREQ)
            $fatal(1, "TickGenerator: require 0 < TICK_FREQ <= CLK_FREQ");
        else if (CLK_FREQ % TICK_FREQ != 0)
            $fatal(1, "TickGenerator: TICK_FREQ must divide CLK_FREQ exactly");
    end
    // synthesis translate_on

    always @(posedge clk) begin
        if (rst) begin
            counter <= 0;
            tick    <= 1'b0;
        end else if (counter == LAST_COUNT) begin
            counter <= 0;
            tick    <= 1'b1;
        end else begin
            counter <= counter + 1'b1;
            tick    <= 1'b0;
        end
    end

    // When TICK_FREQ == CLK_FREQ, tick stays high after reset: every cycle
    // is enabled, with no intervening low cycles.
endmodule
