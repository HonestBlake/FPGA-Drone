`timescale 1ns / 1ps
// Self-contained for the VS Code runner, which runs in this test's folder.
`include "../../../../Drone/PeripheralInterface/RadioReceiver/UartReceiver/UartReceiver.v"
`include "../../../../Drone/PeripheralInterface/RadioReceiver/IBusDecoder/IBusDecoder.v"
`include "../../../../Drone/PeripheralInterface/RadioReceiver/RCMapper/RCMapper.v"
`include "../../../../Drone/PeripheralInterface/RadioReceiver/RadioReceiver.v"
`timescale 1ns / 1ps

module RadioReceiver_tb;
    reg clk = 0;
    reg rst = 1;
    reg ibus_rx = 1;
    wire [9:0] throttle;
    wire signed [10:0] roll_command, pitch_command, yaw_command;
    wire arm_request, avoidance_enable, frame_valid, rc_valid;
    wire [223:0] channels;
    reg [7:0] packet [0:31];
    reg [223:0] expected_channels = 0;
    reg [223:0] previous_channels = 0;
    reg previous_valid = 0;
    reg allow_frame = 0;
    integer frame_count = 0;
    integer error_count = 0;
    integer i;
    integer bit_time = 8681; // Independent 115200 baud transmitter, in ns.

    always #5 clk = ~clk; // Actual Basys3 100 MHz clock.
    RadioReceiver #(.TIMEOUT_MS(5)) dut (
        .clk(clk), .rst(rst), .ibus_rx(ibus_rx), .throttle(throttle),
        .roll_command(roll_command), .pitch_command(pitch_command),
        .yaw_command(yaw_command), .arm_request(arm_request),
        .avoidance_enable(avoidance_enable), .frame_valid(frame_valid),
        .rc_valid(rc_valid), .channels(channels)
    );

    wire [9:0] remapped_throttle;
    wire signed [10:0] remapped_roll;
    wire remapped_arm, remapped_mode;
    RCMapper #(.ROLL_CHANNEL(14), .PITCH_CHANNEL(1), .THROTTLE_CHANNEL(2),
               .YAW_CHANNEL(3), .ARM_CHANNEL(6), .MODE_CHANNEL(5),
               .SWITCH_THRESHOLD(1750)) remapped (
        .channels(channels), .rc_valid(rc_valid), .throttle(remapped_throttle),
        .roll_command(remapped_roll), .pitch_command(), .yaw_command(),
        .arm_request(remapped_arm), .avoidance_enable(remapped_mode)
    );

    initial begin
        $dumpfile("RadioReceiver.vcd");
        // Keep long serial tests compact; +full_wave also records the fast clock
        // and internal counters when deeper debugging is needed.
        $dumpvars(0, rst, ibus_rx, throttle, roll_command, pitch_command,
                  yaw_command, arm_request, avoidance_enable, frame_valid,
                  rc_valid, channels, dut.data, dut.data_valid, dut.framing_error);
        if ($test$plusargs("full_wave")) $dumpvars(0, dut);
    end

    // Check every cycle, including during partial and corrupt frames.
    always @(posedge clk) begin
        #1;
        if (!rst) begin
            if (frame_valid) begin
                if (previous_valid || !allow_frame || channels !== expected_channels || !rc_valid)
                    $fatal(1, "Unexpected frame, wrong channel data, or non-pulsed frame_valid");
                frame_count = frame_count + 1;
            end else if (channels !== previous_channels)
                $fatal(1, "Channels changed without an accepted frame");
            if (!rc_valid && {throttle, roll_command, pitch_command, yaw_command,
                             arm_request, avoidance_enable} !== 45'b0)
                $fatal(1, "Invalid RC must produce zero commands and no arm request");
            if (dut.framing_error) error_count = error_count + 1;
        end
        previous_channels = channels;
        previous_valid = frame_valid;
    end

    task send_byte;
        input [7:0] value;
        input bad_stop;
        integer bit_number;
        begin
            ibus_rx = 0;
            #(bit_time);
            for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1) begin
                ibus_rx = value[bit_number];
                #(bit_time);
            end
            ibus_rx = !bad_stop;
            #(bit_time);
            ibus_rx = 1;
        end
    endtask

    task set_channel;
        input integer channel_number;
        input [15:0] value;
        begin
            packet[channel_number*2] = value[7:0];
            packet[channel_number*2+1] = value[15:8];
        end
    endtask

    task checksum_packet;
        integer byte_number;
        integer sum;
        begin
            sum = 0;
            for (byte_number = 0; byte_number < 30; byte_number = byte_number + 1)
                sum = sum + packet[byte_number];
            sum = 65535 - sum;
            packet[30] = sum & 255;
            packet[31] = (sum >> 8) & 255;
            for (byte_number = 0; byte_number < 28; byte_number = byte_number + 1)
                expected_channels[byte_number*8 +: 8] = packet[byte_number+2];
        end
    endtask

    task send_packet;
        integer byte_number;
        begin
            for (byte_number = 0; byte_number < 32; byte_number = byte_number + 1)
                send_byte(packet[byte_number], 0);
            #100;
        end
    endtask

    task expect_count;
        input integer count;
        begin
            if (frame_count != count)
                $fatal(1, "Expected %0d accepted frames, got %0d", count, frame_count);
        end
    endtask

    initial begin
        #103;
        rst = 0;
        #1000;
        if (rc_valid !== 0) $fatal(1, "RC valid before first packet");

        // False start and unrecognized header/command must not yield a frame.
        ibus_rx = 0;
        #1000;
        ibus_rx = 1;
        #20000;
        send_byte(8'h99, 0);
        send_byte(8'h20, 0);
        send_byte(8'h41, 0);
        expect_count(0);

        // Known checksum vector: fourteen centered channels -> 0xF351.
        packet[0] = 8'h20;
        packet[1] = 8'h40;
        for (i = 1; i <= 14; i = i + 1) set_channel(i, 1500);
        checksum_packet;
        if ({packet[31], packet[30]} !== 16'hf351)
            $fatal(1, "Bad reference packet checksum");
        allow_frame = 1;
        send_packet;
        expect_count(1);
        if (throttle !== 500 || roll_command !== 0 || pitch_command !== 0 || yaw_command !== 0)
            $fatal(1, "Incorrect centered controls");

        // All 14 channels differ, verifying byte order and high-numbered slots.
        for (i = 1; i <= 14; i = i + 1) set_channel(i, 1100 + i*17);
        set_channel(1, 1000);
        set_channel(2, 2000);
        set_channel(3, 1250);
        set_channel(4, 1500);
        set_channel(5, 2000);
        set_channel(6, 1000);
        checksum_packet;
        send_packet; // No inter-frame idle required.
        expect_count(2);
        if (throttle !== 250 || roll_command !== -11'sd500 ||
            pitch_command !== 11'sd500 || yaw_command !== 0 ||
            arm_request !== 1 || avoidance_enable !== 0)
            $fatal(1, "Incorrect mapped controls");
        if (remapped_roll !== -11'sd162 || remapped_throttle !== 1000 ||
            remapped_arm !== 0 || remapped_mode !== 1)
            $fatal(1, "Channel remapping failed");

        // Bad checksums must not commit or refresh the 5 ms test timeout.
        allow_frame = 0;
        packet[30] = packet[30] ^ 8'h01;
        send_packet;
        if (!rc_valid) $fatal(1, "Timeout fired early");
        send_packet;
        expect_count(2);
        if (rc_valid !== 0) $fatal(1, "Corrupt packets refreshed RC timeout");
        packet[30] = packet[30] ^ 8'h01;

        // Truncated frame: a gap permits immediate recovery on a new header.
        for (i = 0; i < 11; i = i + 1) send_byte(packet[i], 0);
        #600000;
        allow_frame = 1;
        send_packet;
        expect_count(3);

        // UART stop-bit error discards the partial frame and waits for idle.
        allow_frame = 0;
        send_byte(8'h20, 0);
        send_byte(8'h40, 0);
        send_byte(8'h55, 1);
        #20000;
        if (error_count != 1) $fatal(1, "Missing framing-error pulse");
        allow_frame = 1;
        send_packet;
        expect_count(4);

        // Clamp controls at the documented limits; exercise faster UART timing.
        set_channel(1, 500);
        set_channel(2, 2500);
        set_channel(3, 2500);
        set_channel(4, 1000);
        set_channel(5, 1000);
        set_channel(6, 2000);
        checksum_packet;
        bit_time = 8500;
        send_packet;
        expect_count(5);
        if (throttle !== 1000 || roll_command !== -11'sd500 ||
            pitch_command !== 11'sd500 || yaw_command !== -11'sd500 ||
            arm_request !== 0 || avoidance_enable !== 1)
            $fatal(1, "Clamping or switch mapping failed");
        bit_time = 8850;
        send_packet;
        expect_count(6);
        bit_time = 8681;

        // Reset during a partial packet must clear validity and live channels.
        allow_frame = 0;
        for (i = 0; i < 9; i = i + 1) send_byte(packet[i], 0);
        @(negedge clk);
        rst = 1;
        #100;
        if (rc_valid !== 0 || channels !== 0 || frame_valid !== 0)
            $fatal(1, "Reset did not clear receiver outputs");
        rst = 0;
        #1000;
        // An extra 0x20 checks overlapping header recovery.
        send_byte(8'h20, 0);
        allow_frame = 1;
        send_packet;
        expect_count(7);

        // A stuck-low wire cannot refresh validity with invented UART bytes.
        allow_frame = 0;
        ibus_rx = 0;
        #5100000;
        if (rc_valid !== 0) $fatal(1, "Stuck-low input did not time out");
        expect_count(7);
        ibus_rx = 1;
        #20000;
        allow_frame = 1;
        send_packet;
        expect_count(8);

        $display("PASS: iBUS serial decode, 14 channels, atomic updates, mapping, clamping,");
        $display("      checksum rejection, timeout, gap/error recovery, reset, and baud tolerance");
        $finish;
    end

    initial begin
        #100000000;
        $fatal(1, "Testbench watchdog expired");
    end
endmodule
