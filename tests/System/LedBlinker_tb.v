`include "../../Drone/System/LedBlinker.v"
`timescale 1ns / 1ps
module LedBlinker_tb;
    reg clk = 0, rst = 1;
    wire led, custom_led;
    integer cycle;
    LedBlinker #(.CLK_FREQ(1000)) dut (.clk(clk), .rst(rst), .led(led));
    LedBlinker #(.CLK_FREQ(1000), .FLASHES_PER_MINUTE(80), .PULSE_MS(50)) custom (
        .clk(clk), .rst(rst), .led(custom_led)
    );
    always #5 clk = ~clk;
    task check_cycles;
        begin
            for (cycle = 0; cycle < 3000; cycle = cycle + 1) begin
                @(negedge clk);
                if (led !== (cycle % 1000 < 100)) $fatal(1, "Default phase %0d", cycle);
                if (custom_led !== (cycle % 750 < 50)) $fatal(1, "Custom phase %0d", cycle);
            end
        end
    endtask
    initial begin
        $dumpfile("LedBlinker.vcd"); $dumpvars(0, LedBlinker_tb);
        repeat (3) @(negedge clk);
        if (led !== 0 || custom_led !== 0) $fatal(1, "Reset off");
        rst = 0; check_cycles;
        rst = 1; @(negedge clk);
        if (led !== 0 || custom_led !== 0) $fatal(1, "Reset during blink");
        rst = 0; check_cycles;
        $display("PASS: LedBlinker default/custom rates, pulse widths, restart");
        $finish;
    end
    initial begin #100000; $fatal(1, "LedBlinker watchdog"); end
endmodule
