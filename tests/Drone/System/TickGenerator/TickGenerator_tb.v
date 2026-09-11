`timescale 1ns / 1ps
`include "../../../../Drone/System/TickGenerator/TickGenerator.v"
`timescale 1ns / 1ps

module TickGenerator_tb;
    reg clk = 1'b0;
    reg rst = 1'b1;
    wire tick_div1, tick_div2, tick_div3, tick_div4;
    integer cycle;
    integer pass;

    always #5 clk = ~clk;

    initial begin
        // Written to the simulation working directory (tests in the runner).
        $dumpfile("TickGenerator.vcd");
        $dumpvars(0, TickGenerator_tb);
    end

    TickGenerator #(.CLK_FREQ(12), .TICK_FREQ(12)) div1
        (.clk(clk), .rst(rst), .tick(tick_div1));
    TickGenerator #(.CLK_FREQ(12), .TICK_FREQ(6)) div2
        (.clk(clk), .rst(rst), .tick(tick_div2));
    TickGenerator #(.CLK_FREQ(12), .TICK_FREQ(4)) div3
        (.clk(clk), .rst(rst), .tick(tick_div3));
    TickGenerator #(.CLK_FREQ(12), .TICK_FREQ(3)) div4
        (.clk(clk), .rst(rst), .tick(tick_div4));

    initial begin
        // Repeat after a mid-period reset to check that phase restarts.
        for (pass = 0; pass < 2; pass = pass + 1) begin
            @(negedge clk);
            rst = 1'b1;
            repeat (2) begin
                @(posedge clk);
                #1;
                if ({tick_div1, tick_div2, tick_div3, tick_div4} !== 4'b0000)
                    $fatal(1, "Tick must be low during reset");
            end
            @(negedge clk);
            rst = 1'b0;
            for (cycle = 1; cycle <= 25; cycle = cycle + 1) begin
                @(posedge clk);
                #1;
                if (tick_div1 !== 1'b1 ||
                    tick_div2 !== (cycle % 2 == 0) ||
                    tick_div3 !== (cycle % 3 == 0) ||
                    tick_div4 !== (cycle % 4 == 0))
                    $fatal(1, "Incorrect tick at pass %0d, cycle %0d", pass, cycle);
            end
        end
        $display("PASS: tick periods 1, 2, 3, 4 and mid-period reset");
        $finish;
    end
endmodule
