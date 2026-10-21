`include "../../Drone/System/SevenSegmentDisplay.v"
`timescale 1ns / 1ps
module SevenSegmentDisplay_tb;
    reg clk = 0, rst = 1, write = 0;
    reg [15:0] value = 0;
    reg [3:0] digit_enable = 0, decimal_points = 0;
    wire [6:0] seg;
    wire dp;
    wire [3:0] an;
    reg [7:0] last_pattern = 8'hff;
    reg [3:0] last_an = 4'hf;
    integer digit, n, visited, active_clocks, blank_clocks;
    reg [6:0] patterns [0:15];
    always #50 clk = ~clk;
    // 10 clocks per digit, 3 clocks blank, 40 clocks per full sweep.
    SevenSegmentDisplay #(.CLK_FREQ(10_000_000), .REFRESH_HZ(250_000), .BLANK_CYCLES(3)) dut (
        .clk(clk), .rst(rst), .write(write), .value(value),
        .digit_enable(digit_enable), .decimal_points(decimal_points), .seg(seg), .dp(dp), .an(an)
    );
    always @(posedge clk) begin
        #1;
        if (!rst) begin
            if (an !== 4'hf && an !== 4'he && an !== 4'hd && an !== 4'hb && an !== 4'h7)
                $fatal(1, "More than one digit enabled");
            if ({dp, seg} !== last_pattern && (an !== 4'hf || last_an !== 4'hf))
                $fatal(1, "Segments changed without prior blanking");
        end
        last_pattern = {dp, seg}; last_an = an;
    end
    task put;
        input [15:0] v;
        input [3:0] en, points;
        begin
            @(negedge clk);
            value = v; digit_enable = en; decimal_points = points; write = 1;
            @(negedge clk); write = 0;
            value = ~v; digit_enable = ~en; decimal_points = ~points;
        end
    endtask
    task check_sweep;
        input [15:0] v;
        input [3:0] en, points;
        integer cycle, index;
        begin
            visited = 0; active_clocks = 0; blank_clocks = 0;
            // Start at the beginning of a known sweep, while all digits are off.
            wait (dut.digit == 0 && dut.counter == 0);
            @(negedge clk);
            for (cycle = 0; cycle < 40; cycle = cycle + 1) begin
                index = cycle / 10;
                if (cycle % 10 < 3 || !en[index]) begin
                    if (an !== 4'hf) $fatal(1, "Blank interval or disabled digit");
                    blank_clocks = blank_clocks + 1;
                end else begin
                    if (an !== ~(4'b0001 << index) || seg !== patterns[v[index*4 +: 4]] || dp !== !points[index])
                        $fatal(1, "Display pattern, digit order, DP or scan duration");
                    visited = visited | (1 << index);
                    active_clocks = active_clocks + 1;
                end
                @(negedge clk);
            end
            if (visited != en || active_clocks + blank_clocks != 40)
                $fatal(1, "Display sweep coverage");
        end
    endtask
    initial begin
        $dumpfile("SevenSegmentDisplay.vcd"); $dumpvars(0, SevenSegmentDisplay_tb);
        // Independent expected segment masks (G..A, active low).
        patterns[0]=7'h40; patterns[1]=7'h79; patterns[2]=7'h24; patterns[3]=7'h30;
        patterns[4]=7'h19; patterns[5]=7'h12; patterns[6]=7'h02; patterns[7]=7'h78;
        patterns[8]=7'h00; patterns[9]=7'h10; patterns[10]=7'h08; patterns[11]=7'h03;
        patterns[12]=7'h46; patterns[13]=7'h21; patterns[14]=7'h06; patterns[15]=7'h0e;
        repeat (3) @(negedge clk);
        if ({an, dp, seg} !== 12'hfff) $fatal(1, "Display reset blanking");
        rst = 0;
        check_sweep(0, 0, 0);
        for (n = 0; n < 16; n = n + 4) begin
            put(((n+3) << 12) | ((n+2) << 8) | ((n+1) << 4) | n, 4'hf, 4'ha);
            // Wait one sweep for the write to become visible.
            repeat (40) @(negedge clk);
            check_sweep(((n+3) << 12) | ((n+2) << 8) | ((n+1) << 4) | n, 4'hf, 4'ha);
        end
        put(16'h1234, 4'h5, 4'h5);
        repeat (40) @(negedge clk); check_sweep(16'h1234, 4'h5, 4'h5);

        // Mid-sweep updates must not affect the remaining old digits.
        put(16'h1111, 4'hf, 0);
        repeat (40) @(negedge clk);
        wait (an == 4'he); // Digit zero of the old sweep is on.
        put(16'h2222, 4'hf, 4'hf);
        wait (an == 4'hd); #1;
        if (seg !== patterns[1] || dp !== 1) $fatal(1, "Display write tore a sweep");
        wait (an == 4'hb); #1;
        if (seg !== patterns[1]) $fatal(1, "Display write tore digit 2");
        wait (an == 4'h7); #1;
        if (seg !== patterns[1]) $fatal(1, "Display write tore digit 3");
        check_sweep(16'h2222, 4'hf, 4'hf);

        put(16'hffff, 0, 4'hf);
        repeat (40) @(negedge clk); check_sweep(16'hffff, 0, 4'hf);
        @(negedge clk); rst = 1;
        @(negedge clk);
        if ({an, dp, seg} !== 12'hfff) $fatal(1, "Display mid-scan reset");
        rst = 0; check_sweep(0, 0, 0);
        $display("PASS: SevenSegmentDisplay hex, order, enable, DP, latching, coherent sweeps, blanking, reset");
        $finish;
    end
    initial begin #200000; $fatal(1, "Display watchdog"); end
endmodule
