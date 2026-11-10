`timescale 1ns / 1ps

// Exercise actual UART packets through the complete top and check physical
// multiplexed display pins. Faster clock/baud parameters keep simulation short.
module RadioDisplayTest_tb;
    reg clk = 0;
    reg btnC = 0;
    reg tx = 1;
    reg [3:0] sw = 0;
    wire [6:0] seg;
    wire dp;
    wire [3:0] an;
    reg [7:0] packet [0:31];
    integer i;
    integer frames = 0;
    always #500 clk = ~clk; // 1 MHz simulation clock.

    RadioDisplayTest #(
        .CLK_FREQ(1_000_000), .BAUD_RATE(100_000),
        .TIMEOUT_MS(100), .DISPLAY_REFRESH_HZ(1000)
    ) dut (.clk(clk), .btnC(btnC), .tx(tx), .sw(sw), .seg(seg), .dp(dp), .an(an));

    always @(posedge clk)
        if (!dut.rst && dut.radio.frame_valid) frames = frames + 1;

    function [6:0] glyph;
        input [3:0] value;
        begin
            case (value)
                0: glyph = 7'h40;
                1: glyph = 7'h79;
                2: glyph = 7'h24;
                3: glyph = 7'h30;
                4: glyph = 7'h19;
                5: glyph = 7'h12;
                6: glyph = 7'h02;
                7: glyph = 7'h78;
                8: glyph = 7'h00;
                9: glyph = 7'h10;
                15: glyph = 7'h3f; // Minus, only segment G lit.
                default: glyph = 7'h7f;
            endcase
        end
    endfunction

    task send_byte;
        input [7:0] value;
        integer b;
        begin
            tx = 0;
            #10000;
            for (b = 0; b < 8; b = b + 1) begin
                tx = value[b];
                #10000;
            end
            tx = 1;
            #10000;
        end
    endtask

    task set_channel;
        input integer channel;
        input [15:0] value;
        begin
            packet[channel*2] = value[7:0];
            packet[channel*2+1] = value[15:8];
        end
    endtask

    task send_packet;
        input corrupt;
        integer n, checksum;
        begin
            checksum = 65535;
            for (n = 0; n < 30; n = n + 1) checksum = checksum - packet[n];
            packet[30] = checksum & 255;
            packet[31] = (checksum >> 8) ^ corrupt;
            @(negedge clk);
            for (n = 0; n < 32; n = n + 1) send_byte(packet[n]);
            repeat (20) @(negedge clk);
        end
    endtask

    task check_display;
        input [3:0] selection;
        input [15:0] expected;
        input visible;
        integer cycle, digit;
        reg [3:0] seen;
        begin
            @(negedge clk) sw = selection;
            // Allow synchronization and two complete display sweeps to settle.
            repeat (2100) @(negedge clk);
            seen = 0;
            for (cycle = 0; cycle < 1100; cycle = cycle + 1) begin
                @(negedge clk);
                if (!visible && an !== 4'hf)
                    $fatal(1, "Expected blank display, selection=%b an=%b", selection, an);
                if (visible && an !== 4'hf) begin
                    case (an)
                        4'b1110: digit = 0;
                        4'b1101: digit = 1;
                        4'b1011: digit = 2;
                        4'b0111: digit = 3;
                        default: $fatal(1, "Invalid anode drive %b", an);
                    endcase
                    seen[digit] = 1;
                    if (seg !== glyph(expected[digit*4 +: 4]) || dp !== 1'b1)
                        $fatal(1, "Selection %b digit %0d expected %h, seg=%b dp=%b",
                               selection, digit, expected[digit*4 +: 4], seg, dp);
                end
            end
            if (visible && seen !== 4'hf) $fatal(1, "Missing display digits: %b", seen);
        end
    endtask

    initial begin
        #250000000;
        $fatal(1, "Simulation watchdog expired");
    end

    initial begin
        // Startup reset works without requiring a button press.
        check_display(4'b0000, 16'hffff, 1);
        check_display(4'b0001, 0, 0);
        if (dut.rst !== 0 || dut.rc_valid !== 0) $fatal(1, "Startup reset failed");
        packet[0] = 8'h20;
        packet[1] = 8'h40;
        for (i = 1; i <= 14; i = i + 1) set_channel(i, 1500);
        set_channel(1, 1250); // Roll -250.
        set_channel(2, 1625); // Pitch +125.
        set_channel(3, 1734); // Throttle 734.
        set_channel(4, 2000); // Yaw +500.
        send_packet(0);
        if (frames != 1 || !dut.rc_valid) $fatal(1, "Good packet not accepted");
        check_display(4'b0001, 16'h0734, 1);
        check_display(4'b0010, 16'hf250, 1);
        check_display(4'b0100, 16'h0125, 1);
        check_display(4'b1000, 16'h0500, 1);
        check_display(4'b0000, 16'hffff, 1);
        check_display(4'b0011, 0, 0);
        check_display(4'b1111, 0, 0);

        set_channel(3, 1000);
        send_packet(1);
        if (frames != 1) $fatal(1, "Bad checksum was accepted");
        check_display(4'b0001, 16'h0734, 1);

        set_channel(1, 1000);
        set_channel(2, 2000);
        set_channel(3, 2000);
        set_channel(4, 1499);
        send_packet(0);
        check_display(4'b0001, 16'h1000, 1);
        check_display(4'b0010, 16'hf500, 1);
        check_display(4'b0100, 16'h0500, 1);
        check_display(4'b1000, 16'hf001, 1);

        set_channel(1, 1500);
        set_channel(2, 1501);
        set_channel(3, 1000);
        set_channel(4, 1500);
        send_packet(0);
        check_display(4'b0001, 16'h0000, 1);
        check_display(4'b0010, 16'h0000, 1);
        check_display(4'b0100, 16'h0001, 1);
        check_display(4'b1000, 16'h0000, 1);

        repeat (100100) @(negedge clk);
        if (dut.rc_valid !== 0) $fatal(1, "Radio did not time out");
        check_display(4'b0000, 16'hffff, 1);
        check_display(4'b0001, 0, 0);
        send_packet(0);
        check_display(4'b0001, 16'h0000, 1);
        @(negedge clk) btnC = 1;
        repeat (20) @(negedge clk);
        if (!dut.rst || dut.rc_valid || an !== 4'hf) $fatal(1, "Button reset failed");
        btnC = 0;
        check_display(4'b0000, 16'hffff, 1);
        check_display(4'b0001, 0, 0);
        send_packet(0);
        check_display(4'b0001, 16'h0000, 1);
        $display("PASS: idle dashes before reception and after timeout/reset, UART reception, four selections, decimal/sign glyphs, limits, invalid selection/checksum, startup and button reset.");
        $finish;
    end
endmodule
