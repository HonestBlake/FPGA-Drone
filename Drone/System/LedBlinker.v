`timescale 1ns / 1ps

// Active-high beacon: default 60 flashes/minute, 100 ms on per flash.
// Same clock domain as the system; reset turns LED off and restarts the phase.
module LedBlinker #(
    parameter integer CLK_FREQ = 100_000_000,
    parameter integer FLASHES_PER_MINUTE = 60,
    parameter integer PULSE_MS = 100
) (
    input wire clk,
    input wire rst,
    output wire led
);
    localparam [63:0] PERIOD_CYCLES = FLASHES_PER_MINUTE > 0 ?
        (64'd60 * CLK_FREQ + FLASHES_PER_MINUTE/2) / FLASHES_PER_MINUTE : 1;
    localparam [63:0] ON_CYCLES = (64'd1 * CLK_FREQ * PULSE_MS + 500) / 1000;
    localparam integer WIDTH = PERIOD_CYCLES > 1 ? $clog2(PERIOD_CYCLES) : 1;
    reg [WIDTH-1:0] counter;
    reg running;
    assign led = running && !rst && counter < ON_CYCLES;

    // synthesis translate_off
    initial begin
        if (CLK_FREQ <= 0 || FLASHES_PER_MINUTE <= 0 || PULSE_MS <= 0 ||
            ON_CYCLES < 1 || ON_CYCLES >= PERIOD_CYCLES)
            $fatal(1, "LedBlinker: require positive clock/rate/pulse and 0 < pulse < period");
    end
    // synthesis translate_on

    always @(posedge clk) begin
        if (rst) begin
            counter <= 0;
            running <= 0;
        end else begin
            running <= 1;
            if (!running || counter == PERIOD_CYCLES - 1) counter <= 0;
            else counter <= counter + 1'b1;
        end
    end
endmodule
