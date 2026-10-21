`include "../../Drone/System/TickGenerator.v"
`include "../../Drone/UartTransmitter.v"
`include "../../Drone/System/UartOutput.v"
`include "../../Drone/System/SevenSegmentDisplay.v"
`include "../../Drone/System/System.v"
`timescale 1ns / 1ps
module System_tb;
    reg clk = 0, rst = 1;
    wire control_tick;
    `include "../support/SystemOutputChecks.vh"
    integer tick_phase = 0;
    always #50 clk = ~clk;
    System #(
        .CLK_FREQ(10_000_000), .CONTROL_TICK_FREQ(1_000_000),
        .UART_BAUD_RATE(1_000_000), .UART_FIFO_DEPTH(2),
        .DISPLAY_REFRESH_HZ(25_000), .DISPLAY_BLANK_CYCLES(3)
    ) dut (
        .clk(clk), .rst(rst), .control_tick(control_tick),
        .uart_data(uart_data), .uart_valid(uart_valid), .uart_ready(uart_ready),
        .uart_busy(uart_busy), .uart_tx(uart_tx), .display_write(display_write),
        .display_value(display_value), .display_digit_enable(display_digit_enable),
        .display_decimal_points(display_decimal_points), .seg(seg), .dp(dp), .an(an)
    );
    always @(posedge clk) begin
        #1;
        if (rst) tick_phase = 0;
        else tick_phase = tick_phase + 1;
        if (control_tick !== (!rst && tick_phase % 10 == 0))
            $fatal(1, "System tick parameter forwarding/reset");
    end
    initial begin
        $dumpfile("System.vcd"); $dumpvars(0, System_tb);
        repeat (3) @(negedge clk);
        rst = 0;
        wait (system_checks_done);
        @(negedge clk); rst = 1;
        @(negedge clk);
        if (control_tick !== 0 || uart_tx !== 1 || uart_busy !== 0 || uart_ready !== 0 ||
            {an, dp, seg} !== 12'hfff) $fatal(1, "System output reset");
        rst = 0;
        repeat (30) @(negedge clk);
        $display("PASS: System tick, UART message, display writes, parameter forwarding, reset");
        $finish;
    end
    initial begin #500000; $fatal(1, "System watchdog"); end
endmodule
