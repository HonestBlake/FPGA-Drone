`timescale 1ns / 1ps

module RadioMotorTest_tb;
    reg clk = 0, btnC = 0, tx = 1;
    reg [3:0] sw = 0;
    wire [3:0] esc, an;
    wire [6:0] seg;
    wire dp;
    wire [7:0] led;
    reg [7:0] packet [0:31];
    integer i, motor, mask;
    always #41.667 clk = ~clk;
    RadioMotorTest #(
        .CLK_FREQ(12_000_000), .BAUD_RATE(1_000_000), .TIMEOUT_MS(10),
        .MOTOR_UPDATE_HZ(12_000), .MOTOR_STARTUP_MS(2),
        .DISPLAY_REFRESH_HZ(12_000)
    ) dut (.clk(clk), .btnC(btnC), .tx(tx), .sw(sw), .esc(esc),
           .seg(seg), .dp(dp), .an(an), .led(led));

    task clocks;
        input integer count;
        integer n;
        begin
            for (n = 0; n < count; n = n + 1) @(negedge clk);
        end
    endtask

    task send_byte;
        input [7:0] value;
        integer b;
        begin
            tx = 0; clocks(12);
            for (b = 0; b < 8; b = b + 1) begin
                tx = value[b]; clocks(12);
            end
            tx = 1; clocks(12);
        end
    endtask

    task send_throttle;
        input [15:0] raw_value;
        input corrupt;
        integer n, checksum;
        begin
            packet[6] = raw_value[7:0]; // CH3 is throttle.
            packet[7] = raw_value[15:8];
            checksum = 65535;
            for (n = 0; n < 30; n = n + 1) checksum = checksum - packet[n];
            packet[30] = checksum & 255;
            packet[31] = (checksum >> 8) ^ corrupt;
            @(negedge clk);
            for (n = 0; n < 32; n = n + 1) send_byte(packet[n]);
            clocks(20);
        end
    endtask

    function [15:0] dshot_packet;
        input [10:0] value;
        reg [11:0] payload;
        begin
            payload = {value, 1'b0};
            dshot_packet = {payload, payload[11:8] ^ payload[7:4] ^ payload[3:0]};
        end
    endfunction

    // Decode the four actual ESC pins, including their checksum and timing.
    // The internal send strobe is used only to align to a whole packet.
    task check_frame;
        input [3:0] active;
        input [10:0] value;
        integer b, cycle, channel;
        integer high_count [0:3];
        reg [15:0] decoded [0:3];
        reg [15:0] expected;
        begin
            @(posedge dut.motors.send);
            @(posedge clk); #1;
            if (esc !== 4'b1111) $fatal(1, "Motor packet starts not synchronized");
            for (channel = 0; channel < 4; channel = channel + 1) decoded[channel] = 0;
            for (b = 15; b >= 0; b = b - 1) begin
                for (channel = 0; channel < 4; channel = channel + 1) high_count[channel] = 0;
                for (cycle = 0; cycle < 40; cycle = cycle + 1) begin
                    for (channel = 0; channel < 4; channel = channel + 1) begin
                        if (esc[channel] === 1) high_count[channel] = high_count[channel] + 1;
                        else if (esc[channel] !== 0) $fatal(1, "Unknown ESC signal");
                    end
                    @(posedge clk); #1;
                end
                for (channel = 0; channel < 4; channel = channel + 1) begin
                    if (high_count[channel] == 30) decoded[channel][b] = 1;
                    else if (high_count[channel] != 15)
                        $fatal(1, "Motor %0d invalid pulse width %0d", channel+1, high_count[channel]);
                end
            end
            for (channel = 0; channel < 4; channel = channel + 1) begin
                expected = dshot_packet(active[channel] ? value : 11'd0);
                if (decoded[channel] !== expected)
                    $fatal(1, "Motor %0d got packet %h expected %h", channel+1, decoded[channel], expected);
            end
            for (cycle = 0; cycle < 80; cycle = cycle + 1) begin
                if (esc !== 0) $fatal(1, "ESC not low during packet gap");
                @(posedge clk); #1;
            end
        end
    endtask

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
                15: glyph = 7'h3f;
                default: glyph = 7'h7f;
            endcase
        end
    endfunction

    task check_display;
        input [15:0] expected;
        integer cycle, digit;
        reg [3:0] seen;
        begin
            clocks(2100);
            seen = 0;
            for (cycle = 0; cycle < 1100; cycle = cycle + 1) begin
                @(negedge clk);
                if (an !== 4'hf) begin
                    case (an)
                        4'b1110: digit = 0;
                        4'b1101: digit = 1;
                        4'b1011: digit = 2;
                        4'b0111: digit = 3;
                        default: $fatal(1, "Invalid display anodes %b", an);
                    endcase
                    seen[digit] = 1;
                    if (seg !== glyph(expected[digit*4 +: 4]) || dp !== 1)
                        $fatal(1, "Display expected %h digit %0d, seg=%h", expected, digit, seg);
                end
            end
            if (seen !== 4'hf) $fatal(1, "Display missing digits");
        end
    endtask

    initial begin
        #100_000_000;
        $fatal(1, "RadioMotorTest watchdog");
    end
    initial begin
        packet[0] = 8'h20;
        packet[1] = 8'h40;
        for (i = 1; i <= 14; i = i + 1) begin
            packet[2*i] = 8'hdc; // 1500 for every channel, including unused arm.
            packet[2*i+1] = 8'h05;
        end
        check_display(16'hffff);
        check_frame(0, 0);
        sw = 1;
        send_throttle(1500, 0);
        check_display(16'h0500);
        while (!led[5]) check_frame(0, 0);
        check_frame(0, 0);
        if (led[4]) $fatal(1, "High throttle enabled drive at startup");
        send_throttle(1000, 0);
        check_frame(0, 0);
        if (!led[4]) $fatal(1, "Idle throttle did not enable selected motor");
        send_throttle(1500, 0);
        check_frame(1, 1047);
        check_display(16'h0500);

        for (motor = 1; motor < 4; motor = motor + 1) begin
            @(negedge clk) sw = 1 << motor;
            clocks(20);
            check_frame(0, 0);
            if (led[4] || led[3:0] !== sw) $fatal(1, "Selection change interlock/status failed");
            send_throttle(1000, 0);
            check_frame(0, 0);
            send_throttle(1500, 0);
            check_frame(1 << motor, 1047);
        end
        send_throttle(1001, 0);
        check_frame(0, 0);
        check_display(16'h0001);
        send_throttle(1009, 0);
        check_frame(0, 0);
        check_display(16'h0009);
        send_throttle(1010, 0);
        check_frame(8, 67);
        check_display(16'h0010);
        send_throttle(2100, 0); // Receiver clamp to 1000.
        check_frame(8, 2047);
        check_display(16'h1000);

        @(negedge clk) sw = 0;
        clocks(20);
        check_frame(0, 0);
        check_display(16'h1000); // Throttle display does not depend on selection.
        @(negedge clk) sw = 3;
        clocks(20);
        check_frame(0, 0);
        if (led[3:0] !== 3 || led[4]) $fatal(1, "New multi-motor selection must await idle");
        send_throttle(1009, 0);
        check_frame(0, 0);
        if (!led[4]) $fatal(1, "Throttle 9 did not enable the selected group");
        send_throttle(1010, 0);
        check_frame(3, 67);
        // Exercise every nonempty selection mask, including all four motors.
        for (mask = 1; mask < 16; mask = mask + 1) begin
            @(negedge clk) sw = mask;
            clocks(20);
            check_frame(0, 0);
            if (led[4]) $fatal(1, "Mask change bypassed idle interlock");
            send_throttle(1009, 0);
            check_frame(0, 0);
            if (!led[4] || led[3:0] !== sw) $fatal(1, "Multi-motor selection/status failed");
            send_throttle(1250, 0);
            check_frame(mask, 547);
        end
        send_throttle(1009, 0);
        check_frame(0, 0); // All four selected: cutoff still stops every motor.
        send_throttle(1010, 0);
        check_frame(15, 67);
        @(negedge clk) sw = 1;
        send_throttle(1000, 1); // Bad idle frame must not enable the new selection.
        check_frame(0, 0);
        if (led[4]) $fatal(1, "Corrupt idle frame enabled motor");
        send_throttle(900, 0); // Clamp below range to zero.
        check_frame(0, 0);
        check_display(16'h0000);
        send_throttle(1734, 0);
        check_frame(1, 1515);
        check_display(16'h0734);
        wait (!led[6]);
        check_frame(0, 0);
        if (led[4]) $fatal(1, "Radio timeout left drive enabled");
        check_display(16'hffff);
        send_throttle(1734, 0);
        check_frame(0, 0);
        send_throttle(1000, 0);
        send_throttle(1734, 0);
        check_frame(1, 1515);

        @(negedge clk) btnC = 1;
        clocks(20);
        if (esc !== 0 || led[4] || led[5] || led[6]) $fatal(1, "Button reset failed");
        btnC = 0;
        check_display(16'hffff);
        send_throttle(1000, 0);
        while (!led[5]) check_frame(0, 0);
        check_frame(0, 0);
        send_throttle(1500, 0);
        check_frame(1, 1047);
        $display("PASS: UART/display, throttle 0..9 cutoff and 10 boundary, all 16 motor masks, DShot checksums/timing, startup, selection interlocks, corrupt packets, clamping, timeout/recovery and reset.");
        $finish;
    end
endmodule
