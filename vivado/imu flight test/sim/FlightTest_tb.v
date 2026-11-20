`timescale 1ns / 1ps
`include "Icm20948Model.vh"
`timescale 1ns / 1ps
module FlightTest_tb;
    reg clk=0, btnC=0, tx=1;
    reg [4:0] sw=0;
    wire [3:0] esc, an;
    wire [6:0] seg;
    wire dp, cs, sclk, mosi, miso;
    wire [15:0] led;
    reg [7:0] packet [0:31];
    integer i;
    always #41.667 clk=~clk;
    FlightTest #(.CLK_FREQ(12_000_000), .CORE_CLK_DIV(1), .BAUD_RATE(1_000_000), .RC_TIMEOUT_MS(10),
        .CONTROL_HZ(12_000), .IMU_POWERUP_CYCLES(20), .IMU_RESET_CYCLES(20),
        .IMU_WAKE_CYCLES(20), .IMU_CALIBRATION_SAMPLES(4), .IMU_TIMEOUT_MS(10),
        .MOTOR_UPDATE_HZ(12_000), .MOTOR_STARTUP_MS(2), .DISPLAY_REFRESH_HZ(12_000)) dut (
        .clk(clk), .btnC(btnC), .tx(tx), .sw(sw), .esc(esc), .seg(seg), .dp(dp), .an(an), .led(led),
        .imu_spi_cs_n(cs), .imu_spi_sclk(sclk), .imu_spi_mosi(mosi), .imu_spi_miso(miso));
    Icm20948Model sensor(dut.rst,cs,sclk,mosi,miso);
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

    task send_frame;
        input corrupt;
        integer n, checksum;
        begin
            checksum = 65535;
            for (n = 0; n < 30; n = n + 1) checksum = checksum - packet[n];
            packet[30] = checksum & 255;
            packet[31] = (checksum >> 8) ^ corrupt;
            @(negedge clk);
            for (n = 0; n < 32; n = n + 1) send_byte(packet[n]);
            clocks(20);
        end
    endtask

    task channel;
        input integer number;
        input [15:0] value;
        begin packet[2*number]=value[7:0]; packet[2*number+1]=value[15:8]; end
    endtask
    task controls;
        input [15:0] thrust, roll, pitch, yaw;
        input arm;
        begin
            channel(1,roll); channel(2,pitch); channel(3,thrust);
            channel(4,yaw); channel(5,1500); // CH5 may stay unconfigured; SW4 controls arming.
            sw[4]=arm;
            send_frame(0);
            clocks(1200);
        end
    endtask
    task rearm;
        input [3:0] mask;
        begin
            @(negedge clk) sw={1'b0,mask};
            controls(1009,1500,1500,1500,0);
            controls(1009,1500,1500,1500,0);
            controls(1009,1500,1500,1500,1);
            if (!led[4] || !led[11] || !led[13]) $fatal(1,"Healthy arm sequence rejected");
            check_frame(0,0,0,0);
        end
    endtask
    task recover_idle;
        integer n;
        begin
            for(n=0;n<32;n=n+1) controls(1000,1500,1500,1500,0);
            if(led[12] || !led[10] || !led[11] || led[4])
                $fatal(1,"Disarmed idle recovery/recalibration failed");
        end
    endtask

    initial begin #200_000_000; $fatal(1,"FlightTest integration watchdog"); end
    initial begin
        packet[0]=8'h20; packet[1]=8'h40;
        for(i=1;i<=14;i=i+1) channel(i,1500);
        sensor.gx=500; // Movement must keep startup calibration incomplete.
        sensor.ready_enable=0;
        check_display(16'hffff);
        sw=15;
        controls(1500,1500,1500,1500,1);
        clocks(110000);
        check_display_mode(16'hc001,4'b0001);
        if(led[15:13] !== 0 || led[10] || led[11] || led[12] || led[4])
            $fatal(1,"Missing initial samples calibration diagnostic");
        sensor.ready_enable=1;
        clocks(110000);
        if(led[10] || led[4]) $fatal(1,"Moving IMU calibrated or armed");
        check_frame(0,0,0,0);
        check_display_mode(16'h4000,4'b0001);
        if(led[15:13] !== 3'b011) $fatal(1,"Calibration freshness/accel/gyro diagnostics");
        sensor.gx=0;
        wait(led[10]);
        clocks(2);
        if(!led[11] || led[12]) $fatal(1,"Calibration/health status failed");
        controls(1500,1500,1500,1500,1);
        if(led[4]) $fatal(1,"High-throttle arm-on bypassed OFF/ON interlock");
        rearm(15);
        channel(5,1000); send_frame(0); clocks(1200);
        if(!led[4] || !led[15]) $fatal(1,"Radio CH5 low changed SW4 arming");
        channel(5,2000); send_frame(0); clocks(1200);
        if(!led[4]) $fatal(1,"Radio CH5 high changed SW4 arming");
        check_frame(0,0,0,0);
        controls(1500,1500,1500,1500,1);
        check_frame(1047,1047,1047,1047);
        check_display(16'h0500);
        @(negedge clk) sw[4]=0; // Must stop even without waiting for another radio packet.
        clocks(8);
        if(led[4] || led[15]) $fatal(1,"SW4 OFF failed to disarm immediately");
        check_frame(0,0,0,0);
        channel(5,2000); send_frame(0); clocks(1200);
        if(led[4]) $fatal(1,"CH5 armed while SW4 OFF");
        rearm(15); controls(1500,1500,1500,1500,1);
        controls(1500,2000,1500,1500,1);
        if(dut.roll_setpoint != 2000) $fatal(1,"Right roll target failed");
        check_frame(797,1297,797,1297);
        controls(1500,1000,1500,1500,1);
        check_frame(1297,797,1297,797);
        controls(1500,1500,2000,1500,1);
        if(dut.pitch_setpoint != -2000) $fatal(1,"Forward pitch target failed");
        check_frame(797,797,1297,1297);
        controls(1500,1500,1000,1500,1);
        if(dut.pitch_setpoint != 2000) $fatal(1,"Reverse pitch target failed");
        check_frame(1297,1297,797,797);
        controls(1500,1500,1500,1750,1);
        check_frame(907,1187,1187,907);
        controls(1500,1500,1500,1250,1);
        check_frame(1187,907,907,1187);
        controls(1500,1750,1250,1750,1);
        check_frame(907,1437,937,907);
        controls(1009,2000,1000,1750,1);
        check_frame(0,0,0,0); // Cutoff overrides all attitude/rate corrections.
        check_display(16'h0009);

        // User's position-to-wire mapping must survive mixer integration.
        rearm(1); controls(1500,1500,1500,1500,1);
        check_frame(0,0,0,1047); // SW0/JA7 is back-left.
        rearm(5); controls(1500,1500,1500,1500,1);
        check_frame(0,0,1047,1047);
        rearm(8); controls(1500,1500,1500,1500,1);
        check_frame(1047,0,0,0); // SW3/JA10 is front-right.
        @(negedge clk) sw=0;
        clocks(20); check_frame(0,0,0,0);
        if(led[4]) $fatal(1,"No selection failed to disarm");
        rearm(15); controls(1500,1500,1500,1500,1);
        sensor.gx=1310;
        @(posedge dut.imu_sample_valid); clocks(1500);
        if(!(dut.fc_fr > dut.fc_fl && dut.fc_br > dut.fc_bl))
            $fatal(1,"Physical SPI gyro feedback did not oppose positive roll rate");
        sensor.gx=0;
        sensor.gy=-1310; // Selected FLU profile: negative sensor Y rate raises the nose.
        @(posedge dut.imu_sample_valid); clocks(1500);
        if(dut.pitch_rate != 1000 || !(dut.fc_fr < dut.fc_br && dut.fc_fl < dut.fc_bl))
            $fatal(1,"FLU-profile pitch feedback did not oppose nose-up rotation");
        sensor.gy=0;
        sensor.gz=-1310; // Selected FLU profile: negative sensor Z rate turns right.
        @(posedge dut.imu_sample_valid); clocks(1500);
        if(dut.yaw_rate != 1000 || !(dut.fc_fr > dut.fc_fl && dut.fc_bl > dut.fc_br))
            $fatal(1,"FLU-profile yaw feedback did not oppose right rotation");
        sensor.gz=0;
        sensor.disconnected=1;
        wait(led[12]);
        clocks(20); check_frame(0,0,0,0);
        if(led[4] || led[11]) $fatal(1,"IMU disconnect did not disarm");
        check_display_mode(16'h00ff,4'b0100);
        if(dut.imu_fault_expected != 8'hea) $fatal(1,"Fault expected-byte forwarding");
        controls(1000,1500,1500,1500,0);
        controls(1000,1500,1500,1500,1);
        if(led[4]) $fatal(1,"SW4 rearmed a faulted IMU");
        sensor.disconnected=0;
        @(negedge clk) btnC=1; clocks(20);
        if(esc !== 0 || led[10] || led[4]) $fatal(1,"Reset failed");
        btnC=0;
        wait(led[10]); rearm(15);
        controls(1500,1500,1500,1500,1);
        check_frame(1047,1047,1047,1047);
        wait(!led[6]); clocks(20); check_frame(0,0,0,0);
        if(led[4]) $fatal(1,"Radio timeout failed to disarm");
        check_display(16'hffff);
        controls(1009,1500,1500,1500,1);
        if(led[4]) $fatal(1,"Radio reconnect bypassed explicit rearm");
        rearm(15);
        controls(1500,1500,1500,1500,1);
        sensor.ready_enable=0;
        wait(led[12]); clocks(20); check_frame(0,0,0,0);
        if(led[4]) $fatal(1,"Stale IMU data failed to disarm");
        check_display_mode(16'he001,4'b0100);
        sensor.ready_enable=1;
        controls(1000,1500,1500,1500,1);
        clocks(100000);
        if(!led[12] || led[4]) $fatal(1,"Fault recovered with SW4 ON");
        controls(1500,1500,1500,1500,0);
        clocks(6000);
        if(!led[12] || led[4]) $fatal(1,"Fault recovered above idle throttle");
        recover_idle;
        controls(1500,1500,1500,1500,0);
        check_frame(0,0,0,0); // Clearing a fault must not resume motors.
        rearm(15); controls(1500,1500,1500,1500,1);
        check_frame(1047,1047,1047,1047);
        sensor.gx=32767;
        wait(led[12]); check_frame(0,0,0,0);
        check_display_mode(16'he002,4'b0100);
        sensor.gx=0; recover_idle;
        rearm(15); controls(1500,1500,1500,1500,1);
        sensor.az=32767;
        wait(led[12]); check_frame(0,0,0,0);
        check_display_mode(16'he003,4'b0100);
        sensor.az=16384; recover_idle;
        sensor.who=0;
        @(negedge clk) btnC=1; clocks(20); btnC=0;
        wait(led[12]);
        if(led[8] || led[9] || led[10] || led[11] || led[4])
            $fatal(1,"Startup identity failure bypassed health gating");
        check_display_mode(16'h0000,4'b0100);
        sensor.who=8'hea;
        $display("PASS: physical SPI calibration/freshness, UART full roll/pitch/yaw, mixer-to-wire mapping, DShot/display, cutoff, selections, arming, watchdogs, faults and reset.");
        $finish;
    end


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
        input [10:0] fr, fl, br, bl;
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
                case (channel)
                    0: expected = dshot_packet(bl);
                    1: expected = dshot_packet(fl);
                    2: expected = dshot_packet(br);
                    3: expected = dshot_packet(fr);
                endcase
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
                10: glyph = 7'h08;
                11: glyph = 7'h03;
                12: glyph = 7'h46;
                13: glyph = 7'h21;
                14: glyph = 7'h06;
                15: glyph = 7'h0e;
                default: glyph = 7'h7f;
            endcase
        end
    endfunction

    task check_display;
        input [15:0] expected;
        begin check_display_mode(expected,4'b0000); end
    endtask

    task check_display_mode;
        input [15:0] expected;
        input [3:0] points;
        integer cycle, digit;
        reg [3:0] seen;
        reg [6:0] expected_seg;
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
                    expected_seg = (expected == 16'hffff && points == 0) ? 7'h3f :
                        glyph(expected[digit*4 +: 4]);
                    if (seg !== expected_seg || dp !== !points[digit])
                        $fatal(1, "Display expected %h digit %0d, seg=%h", expected, digit, seg);
                end
            end
            if (seen !== 4'hf) $fatal(1, "Display missing digits");
        end
    endtask



endmodule
