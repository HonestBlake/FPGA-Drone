`include "../../../tests/support/XadcModel.vh"
`timescale 1ns / 1ps
`include "Icm20948Model.vh"
`timescale 1ns / 1ps
module BatteryLightsFlightTest_tb;
    reg clk=0, btnC=0, tx=1;
    reg sonar_echo=0; wire sonar_trigger;
    reg [15:0] sw=0;
    wire [2:0] light_out;
    wire [3:0] esc, an;
    wire [6:0] seg;
    wire dp, cs, sclk, mosi, miso;
    wire [15:0] led;
    reg [7:0] packet [0:31];
    integer i;
    always #41.667 clk=~clk;
    BatteryLightsFlightTest #(.CLK_FREQ(12_000_000), .CORE_CLK_DIV(1), .BAUD_RATE(1_000_000), .RC_TIMEOUT_MS(10),
        .BATTERY_FILTER_SAMPLES(1), .BATTERY_UPDATE_HZ(12_000_000), .YAW_HEADING_HOLD(0), .ENABLE_HOVER(0), .ENABLE_POSITION_DIAGNOSTICS(1), .CONTROL_HZ(12_000), .IMU_POWERUP_CYCLES(20), .IMU_RESET_CYCLES(20),
        .IMU_WAKE_CYCLES(20), .IMU_CALIBRATION_SAMPLES(4), .IMU_TIMEOUT_MS(10),
        .MOTOR_UPDATE_HZ(12_000), .MOTOR_STARTUP_MS(2), .DISPLAY_REFRESH_HZ(12_000)) dut (
        .ultrasonic_echo(sonar_echo), .ultrasonic_trigger(sonar_trigger), .lidar_rx(1'b1), .lidar_tx(), .usb_uart_tx(), .battery_vin_p(1'b0), .battery_vin_n(1'b0), .light_out(light_out),
        .clk(clk), .btnC(btnC), .tx(tx), .sw(sw), .esc(esc), .seg(seg), .dp(dp), .an(an), .led(led),
        .imu_spi_cs_n(cs), .imu_spi_sclk(sclk), .imu_spi_mosi(mosi), .imu_spi_miso(miso));
    always begin
        @(negedge sonar_trigger);
        if(!dut.rst) begin #10000; sonar_echo=1; #3600000; sonar_echo=0; end
    end
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
            channel(4,yaw); channel(5,1500); // CH5 may stay unconfigured; SW15 controls arming.
            sw[15]=arm;
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
            check_display(16'hadbf);
            controls(1009,1500,1500,1500,1);
            if (!dut.fc_armed || !dut.imu_valid || !dut.fc_command_valid) $fatal(1,"Healthy arm sequence rejected");
            check_frame(0,0,0,0);
        end
    endtask
    task recover_idle;
        integer n;
        begin
            for(n=0;n<32;n=n+1) controls(1000,1500,1500,1500,0);
            if(dut.imu_fault || !dut.imu_calibrated || !dut.imu_valid || dut.fc_armed)
                $fatal(1,"Disarmed idle recovery/recalibration failed");
        end
    endtask

    task check_status_leds;
        input phase;
        reg [15:0] expected;
        begin
            force dut.heartbeat=phase;
            #1;
            expected=0;
            expected[3:0]=dut.sw_sync | {4{phase}};
            expected[11]=dut.ai_override_request; expected[12]=dut.hover_request;
            expected[15]=dut.fc_armed || (dut.fc_can_arm && phase);
            expected[8]=(dut.imu_valid && dut.imu_calibrated && !dut.imu_fault) || phase;
            expected[7]=dut.rc_valid || phase;
            expected[13]=dut.battery_valid && (dut.battery_percent>20 || phase);
            expected[14]=dut.error_view || (dut.error_present && phase);
            expected[9]=dut.lidar_healthy || phase; expected[10]=dut.ultrasonic_valid || phase;
            expected[6]=dut.light_sync[0]; expected[5]=dut.light_sync[1];
            expected[4]=dut.light_sync[2] && phase;
            if(light_out !== {dut.light_sync[2] && phase,dut.light_sync[1:0]})
                $fatal(1,"External lights blink/on-off mismatch");
            if(led !== expected) $fatal(1,"Status LEDs got %h expected %h",led,expected);
            release dut.heartbeat;
        end
    endtask

    initial begin #200_000_000; $fatal(1,"BatteryLightsFlightTest integration watchdog"); end
    initial begin
        dut.battery.xadc.sample_code=3226;
        packet[0]=8'h20; packet[1]=8'h40;
        for(i=1;i<=14;i=i+1) channel(i,1500);
        sensor.gx=500; // Movement must keep startup calibration incomplete.
        sensor.ready_enable=0;
        sw=16'h6780; // Every display selector on: configuration must still win.
        check_display(16'hc0f6); check_status_leds(0); check_status_leds(1);
        sw=15;
        controls(1500,1500,1500,1500,1);
        clocks(110000);
        check_display(16'hc0f6); check_status_leds(0); check_status_leds(1);
        if({dut.gyro_window,dut.accel_window,dut.raw_fresh} !== 0 || dut.imu_calibrated || dut.imu_valid || dut.imu_fault || dut.fc_armed)
            $fatal(1,"Missing initial samples calibration diagnostic");
        sensor.ready_enable=1;
        clocks(110000);
        if(dut.imu_calibrated || dut.fc_armed) $fatal(1,"Moving IMU calibrated or armed");
        check_frame(0,0,0,0);
        check_display(16'hc0f6); check_status_leds(0); check_status_leds(1);
        if({dut.gyro_window,dut.accel_window,dut.raw_fresh} !== 3'b011) $fatal(1,"Calibration freshness/accel/gyro diagnostics");
        sensor.gx=0;
        wait(dut.imu_calibrated);
        clocks(2);
        if(!dut.imu_valid || dut.imu_fault) $fatal(1,"Calibration/health status failed");
        controls(1500,1500,1500,1500,1);
        if(dut.fc_armed) $fatal(1,"High-throttle arm-on bypassed OFF/ON interlock");
        rearm(15);
        channel(5,1000); send_frame(0); clocks(1200);
        if(!dut.fc_armed || !dut.arm_request) $fatal(1,"Radio CH5 low changed SW15 arming");
        channel(5,2000); send_frame(0); clocks(1200);
        if(!dut.fc_armed) $fatal(1,"Radio CH5 high changed SW15 arming");
        check_frame(0,0,0,0);
        controls(1500,1500,1500,1500,1);
        check_frame(1047,1047,1047,1047);
        channel(7,2000); channel(10,2000); send_frame(0); clocks(1200);
        if(!led[11] || !led[12]) $fatal(1,"CH7/10 requests not visible");
        check_frame(1047,1047,1047,1047); // Future mode requests cannot change motors.
        channel(7,1000); channel(10,1000); send_frame(0); clocks(1200);
        check_display(16'ha17d); sw[7]=1; clocks(6); check_display(16'hc000); check_status_leds(0); check_status_leds(1);
        sw[8]=1; clocks(6);
        force dut.roll_cd=16'sd2000; check_display(16'h0020);
        force dut.roll_cd=-16'sd2000; check_display(16'h0020);
        release dut.roll_cd; sw[8]=0; clocks(6);
        sw[13]=1; clocks(6); check_display_mode(16'h1000,4'b0010);
        dut.battery.xadc.sample_code=2957; clocks(50);
        check_display_mode(16'h0500,4'b0010);
        dut.battery.xadc.sample_code=2744; clocks(50);
        check_display_mode(16'h0104,4'b0010); check_status_leds(0); check_status_leds(1);
        dut.battery.xadc.sample_code=2688; clocks(50);
        check_display_mode(16'h0000,4'b0010); check_status_leds(0); check_status_leds(1);
        dut.battery.xadc.sample_code=3226; clocks(50);
        force dut.battery_valid=0;
        sw[14]=1; clocks(6); check_display(16'he011);
        check_status_leds(0); check_status_leds(1);
        sw[14]=0; clocks(6);
        release dut.battery_valid;
        sw[14]=1; clocks(6); check_display(16'he000);
        check_status_leds(0); check_status_leds(1);
        sw[14]=0; sw[13]=0; clocks(6);
        @(negedge clk) sw[15]=0; // Must stop even without waiting for another radio packet.
        clocks(8);
        if(dut.fc_armed || dut.arm_request) $fatal(1,"SW15 OFF failed to disarm immediately");
        check_frame(0,0,0,0);
        channel(5,2000); send_frame(0); clocks(1200);
        if(dut.fc_armed) $fatal(1,"CH5 armed while SW15 OFF");
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
        sw[7]=1; clocks(6); check_display(16'hc250); check_status_leds(0); check_status_leds(1);

        // User's position-to-wire mapping must survive mixer integration.
        rearm(1); controls(1500,1500,1500,1500,1);
        check_frame(0,0,0,1047); // SW0/JB1 is back-left.
        rearm(5); controls(1500,1500,1500,1500,1);
        check_frame(0,0,1047,1047);
        rearm(8); controls(1500,1500,1500,1500,1);
        check_frame(1047,0,0,0); // SW3/JB4 is front-right.
        rearm(2); controls(1500,1500,1500,1500,1);
        check_frame(0,1047,0,0); // SW1/JB2 is front-left.
        @(negedge clk) sw=0;
        clocks(20); check_frame(0,0,0,0);
        if(dut.fc_armed) $fatal(1,"No selection failed to disarm");
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
        wait(dut.imu_fault);
        clocks(20); check_frame(0,0,0,0);
        if(dut.fc_armed || dut.imu_valid) $fatal(1,"IMU disconnect did not disarm");
        check_display_mode(16'h00ff,4'b0100); check_status_leds(0); check_status_leds(1);
        if(dut.imu_fault_expected != 8'hea) $fatal(1,"Fault expected-byte forwarding");
        controls(1000,1500,1500,1500,0);
        controls(1000,1500,1500,1500,1);
        if(dut.fc_armed) $fatal(1,"SW4 rearmed a faulted IMU");
        sensor.disconnected=0;
        @(negedge clk) btnC=1; clocks(20);
        if(esc !== 0 || dut.imu_calibrated || dut.fc_armed) $fatal(1,"Reset failed");
        btnC=0;
        wait(dut.imu_calibrated); rearm(15);
        controls(1500,1500,1500,1500,1);
        check_frame(1047,1047,1047,1047);
        wait(!dut.rc_valid); clocks(20); check_frame(0,0,0,0);
        if(dut.fc_armed) $fatal(1,"Radio timeout failed to disarm");
        check_display(16'hffff); check_status_leds(0); check_status_leds(1);
        sw[14]=1; clocks(6); check_display(16'he010);
        sw[14]=0; clocks(6);
        controls(1009,1500,1500,1500,1);
        if(dut.fc_armed) $fatal(1,"Radio reconnect bypassed explicit rearm");
        rearm(15);
        controls(1500,1500,1500,1500,1);
        sensor.ready_enable=0;
        wait(dut.imu_fault); clocks(20); check_frame(0,0,0,0);
        if(dut.fc_armed) $fatal(1,"Stale IMU data failed to disarm");
        check_display_mode(16'he001,4'b0100);
        sensor.ready_enable=1;
        controls(1000,1500,1500,1500,1);
        clocks(100000);
        if(!dut.imu_fault || dut.fc_armed) $fatal(1,"Fault recovered with SW4 ON");
        controls(1500,1500,1500,1500,0);
        clocks(6000);
        if(!dut.imu_fault || dut.fc_armed) $fatal(1,"Fault recovered above idle throttle");
        recover_idle;
        controls(1500,1500,1500,1500,0);
        check_frame(0,0,0,0); // Clearing a fault must not resume motors.
        rearm(15); controls(1500,1500,1500,1500,1);
        check_frame(1047,1047,1047,1047);
        sensor.gx=32767;
        wait(dut.imu_fault); check_frame(0,0,0,0);
        check_display_mode(16'he002,4'b0100);
        sensor.gx=0; recover_idle;
        rearm(15); controls(1500,1500,1500,1500,1);
        sensor.az=32767;
        wait(dut.imu_accel_clipped || dut.imu_fault); clocks(1500);
        if(dut.imu_fault || !dut.imu_valid || !dut.fc_armed || !dut.imu_accel_clipped)
            $fatal(1,"Accel impact disarmed healthy gyro stabilization");
        check_frame(1047,1047,1047,1047);
        sensor.az=16384;
        wait(!dut.imu_accel_clipped); clocks(1500);
        if(dut.imu_accel_clipped || !dut.fc_armed) $fatal(1,"Accel impact did not recover without rearm");
        sensor.who=0;
        @(negedge clk) btnC=1; clocks(20); btnC=0;
        wait(dut.imu_fault);
        if(dut.imu_detected || dut.imu_init_done || dut.imu_calibrated || dut.imu_valid || dut.fc_armed)
            $fatal(1,"Startup identity failure bypassed health gating");
        check_display_mode(16'h0000,4'b0100);
        sensor.who=8'hea;
        // Reset bus fault before checking battery/lighting independently of radio.
        btnC=1; clocks(5); btnC=0; clocks(50);
        // Real UART packets: all 3x2x2x2 light and future-request combinations.
        sw[15]=0; sw[6:4]=7; // Diagnostic selectors must not alter radio-controlled lights.
        for(i=0;i<24;i=i+1) begin
            channel(7,((i/6)%2) ? 2000 : 1000);
            channel(8,((i/3)%2) ? 2000 : 1000);
            channel(9,1000+500*(i%3));
            channel(10,((i/12)%2) ? 2000 : 1000);
            controls(1000,1500,1500,1500,0);
            check_status_leds(0); check_status_leds(1);
            if(dut.light_sync[0] !== ((i%3)>0) || dut.light_sync[1] !== ((i%3)==2) ||
               dut.light_sync[2] !== (((i/3)%2)!=0)) $fatal(1,"CH8/9 physical light mapping");
            if(led[11] !== (((i/6)%2)!=0) || led[12] !== (((i/12)%2)!=0))
                $fatal(1,"CH7/10 request mapping");
            if(dut.fc_armed) $fatal(1,"Aux requests must not arm motors");
        end
        channel(7,1000);channel(8,1000);channel(9,1000);channel(10,1000);send_frame(0);clocks(20);
        channel(8,2000);channel(9,2000);send_frame(1);clocks(20);
        if(dut.light_sync!==0) $fatal(1,"Corrupt packet changed lights");
        send_frame(0);clocks(20);
        if(dut.light_sync!==7) $fatal(1,"Valid auxiliary packet not accepted");
        channel(8,2500);channel(9,500);send_frame(0);clocks(20);
        if(dut.light_sync!==0) $fatal(1,"Invalid auxiliary values enabled lights");
        channel(7,2000);channel(8,2000);channel(9,2000);channel(10,2000);send_frame(0);clocks(20);
        wait(!dut.rc_valid);clocks(2);
        if(light_out!==0 || led[11] || led[12]) $fatal(1,"Radio timeout did not clear aux outputs");
        sw[4]=1; clocks(6);
        if(dut.arm_request) $fatal(1,"SW4 diagnostic must not arm");
        sw[6:4]=0;clocks(6); // Clear every diagnostic selector before testing position views.
        dut.battery.xadc.sample_code=2688; clocks(50);
        if(dut.battery_percent!==0) $fatal(1,"Battery empty endpoint");
        dut.battery.xadc.sample_code=2957; clocks(50);
        if(dut.battery_percent!==50) $fatal(1,"Battery midpoint");
        dut.battery.xadc.sample_code=3226; clocks(50);
        if(dut.battery_percent!==100) $fatal(1,"Battery full endpoint");
        force dut.position_valid=1;
        force dut.position_x_mm=-32'sd1230;force dut.position_y_mm=32'sd4560;
        sw[11]=1;clocks(6);check_display(16'h0123);
        sw[11]=0;sw[12]=1;clocks(6);check_display(16'h0456);
        release dut.position_valid;release dut.position_x_mm;release dut.position_y_mm;
        sw[12]=0;
        btnC=1; clocks(5);
        if(light_out!==0) $fatal(1,"Reset must turn external lights off");
        $display("PASS: battery scaling/endpoints, radio auxiliary switches/lights/requests, SW15 arm,  physical SPI calibration/freshness, UART full roll/pitch/yaw, mixer-to-wire mapping, DShot/display, cutoff, selections, arming, watchdogs, faults and reset.");
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
            if(points == 4'b0100) begin
                check_status_leds(0); check_status_leds(1);
                sw[14]=1; clocks(6);
            end
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
                    if(dut.roll_view_active && digit==3)
                        expected_seg=dut.roll_cd[15] ? 7'h3f : 7'h7f;
                    if(digit==3 && (expected[15:12]==4'hc || expected[15:12]==4'hd) &&
                       expected[11:0]<=12'h500 && expected[7:4]<=9 && expected[3:0]<=9)
                        expected_seg=expected[15:12]==4'hd ? 7'h3f : 7'h7f;
                    if(dut.position_view_active && digit==3)
                        expected_seg=dut.position_value_mm[31] ? 7'h3f : 7'h7f;
                    if(expected == 16'hc0f6) begin
                        case(digit)
                            3: expected_seg=7'h46;
                            2: expected_seg=7'h2b;
                            1: expected_seg=7'h0e;
                            0: expected_seg=7'h42;
                        endcase
                    end
                    if(expected == 16'ha17d) begin
                        case(digit)
                            3: expected_seg=7'h08;
                            2: expected_seg=7'h2f;
                            1: expected_seg=7'h2b;
                            0: expected_seg=7'h21;
                        endcase
                    end
                    if(expected == 16'hadbf) begin
                        case(digit)
                            3: expected_seg=7'h7f;
                            2: expected_seg=7'h2f;
                            1: expected_seg=7'h21;
                            0: expected_seg=7'h11;
                        endcase
                    end
                    if (seg !== expected_seg || dp !== !points[digit])
                        $fatal(1, "Display expected %h digit %0d, seg=%h", expected, digit, seg);
                end
            end
            if (seen !== 4'hf) $fatal(1, "Display missing digits");
            if(points == 4'b0100) begin sw[14]=0; clocks(6); end
        end
    endtask



endmodule
