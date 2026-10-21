`include "../../../Drone/PeripheralInterface/UartReceiver.v"
`include "../../../Drone/PeripheralInterface/IMU/imu_spi.v"
`include "../../../Drone/PeripheralInterface/IMU/ImuEstimator.v"
`include "../../../Drone/PeripheralInterface/IMU/IMU.v"
`include "../../../Drone/UartTransmitter.v"
`include "../../../Drone/PeripheralInterface/LidarArray/LidarArray.v"
`include "../../../Drone/PeripheralInterface/RadioReceiver/IBusDecoder.v"
`include "../../../Drone/PeripheralInterface/RadioReceiver/RCMapper.v"
`include "../../../Drone/PeripheralInterface/RadioReceiver/RadioReceiver.v"
`include "../../../Drone/PeripheralInterface/PeripheralInterface.v"
`include "../../../Drone/System/TickGenerator.v"
`include "../../../Drone/System/UartOutput.v"
`include "../../../Drone/System/SevenSegmentDisplay.v"
`include "../../../Drone/System/System.v"
`include "../../../Drone/NPU/NPU.v"
`include "../../../Drone/FlightController/FlightController.v"
`include "../../../Drone/PeripheralInterface/MotorControl/MotorDriver.v"
`include "../../../Drone/PeripheralInterface/MotorControl/MotorControl.v"
`include "../../../Drone/Drone.v"
`timescale 1ns / 1ps
module LidarArray_tb;
    // Use real 115200-baud bit periods, intentionally not the DUT's rounded
    // divider (35 clocks at 4 MHz), to exercise independent UART timing.
    // Use enough clock resolution for the enclosing Drone's DShot300 output.
    localparam integer BIT_NS = 8681;
    reg clk = 0, rst = 1, lidar_rx = 1;
    reg [5:0] read_address = 0;
    wire lidar_tx, image_valid, frame_valid, initialized, error;
    wire [15:0] distance_mm;
    wire [31:0] frame_count;
    wire top_tx, top_valid, top_frame, top_initialized, top_error;
    wire [15:0] top_distance;
    wire [31:0] top_count;
    integer frames = 0, errors = 0, old_errors, n;
    reg previous_frame = 0, previous_error = 0;
    reg [7:0] command_bytes [0:255];
    integer command_count = 0, command_read = 0;
    reg [7:0] command_byte;
    integer b;
    always #125 clk = ~clk;
    LidarArray #(
        .CLK_FREQ(4_000_000), .STARTUP_MS(1), .SETTLE_MS(2), .POLL_MS(1),
        .RESPONSE_TIMEOUT_MS(30), .BYTE_TIMEOUT_MS(1)
    ) dut (
        .clk(clk), .rst(rst), .lidar_rx(lidar_rx), .lidar_tx(lidar_tx),
        .read_address(read_address), .read_distance_mm(distance_mm),
        .image_valid(image_valid), .frame_valid(frame_valid),
        .frame_count(frame_count), .initialized(initialized), .error(error)
    );
    // Exercise both enclosing modules with the same serial traffic and
    // non-default timing parameters; compare every public output every cycle.
    Drone #(
        .CLK_FREQ(4_000_000), .LIDAR_STARTUP_MS(1), .LIDAR_SETTLE_MS(2),
        .LIDAR_POLL_MS(1), .LIDAR_RESPONSE_TIMEOUT_MS(30), .LIDAR_BYTE_TIMEOUT_MS(1)
    ) top_dut (
        .clk(clk), .rst(rst), .ibus_rx(1'b1), .imu_spi_miso(1'b0),
        .imu_sample_valid(1'b0), .imu_valid(1'b0), .imu_calibrated(1'b0), .imu_fault(1'b0),
        .imu_roll_cd(16'sd0), .imu_pitch_cd(16'sd0),
        .imu_roll_rate_cdps(16'sd0), .imu_pitch_rate_cdps(16'sd0), .imu_yaw_rate_cdps(16'sd0),
        .npu_external_available(1'b0), .npu_external_action_valid(1'b0), .npu_external_action(3'd0),
        .lidar_rx(lidar_rx), .lidar_tx(top_tx), .lidar_read_address(read_address),
        .lidar_distance_mm(top_distance), .lidar_image_valid(top_valid),
        .lidar_frame_valid(top_frame), .lidar_frame_count(top_count),
        .lidar_initialized(top_initialized), .lidar_error(top_error),
        .uart_data(8'd0), .uart_valid(1'b0), .display_write(1'b0),
        .display_value(16'd0), .display_digit_enable(4'd0), .display_decimal_points(4'd0)
    );
    always @(posedge clk) begin
        #1;
        if ({top_tx, top_valid, top_frame, top_count, top_initialized, top_error, top_distance} !==
            {lidar_tx, image_valid, frame_valid, frame_count, initialized, error, distance_mm})
            $fatal(1, "Drone/PeripheralInterface lidar routing or parameters");
        if (!rst) begin
            if (frame_valid) begin
                if (previous_frame) $fatal(1, "Frame pulse longer than one clock");
                frames = frames + 1;
            end
            if (error) begin
                if (previous_error) $fatal(1, "Error pulse longer than one clock");
                errors = errors + 1;
            end
        end
        previous_frame = frame_valid;
        previous_error = error;
    end
    // Independent bit-level command decoder, with the nominal sensor baud.
    initial forever begin
        @(negedge lidar_tx);
        if (!rst) begin
            #(BIT_NS/2);
            if (lidar_tx !== 0) $fatal(1, "Command start bit");
            for (b = 0; b < 8; b = b + 1) begin
                #BIT_NS; command_byte[b] = lidar_tx;
            end
            #BIT_NS;
            if (lidar_tx !== 1) $fatal(1, "Command stop bit");
            command_bytes[command_count] = command_byte;
            command_count = command_count + 1;
        end
    end
    task expect_request;
        input mode;
        integer j, count;
        reg [7:0] expected;
        begin
            count = mode ? 8 : 4;
            wait (command_count >= command_read + count);
            for (j = 0; j < count; j = j + 1) begin
                case (j)
                    0: expected = 8'h55;
                    2: expected = mode ? 5 : 1;
                    3: expected = mode ? 1 : 2;
                    7: expected = 8;
                    default: expected = 0;
                endcase
                if (command_bytes[command_read+j] !== expected)
                    $fatal(1, "Request byte %0d: got %h expected %h", j,
                           command_bytes[command_read+j], expected);
            end
            command_read = command_read + count;
            #(BIT_NS*2); // Finish the command stop bit before responding.
        end
    endtask
    task send_byte;
        input [7:0] value;
        input bad_stop;
        integer bit_number;
        begin
            lidar_rx = 0; #BIT_NS;
            for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1) begin
                lidar_rx = value[bit_number]; #BIT_NS;
            end
            lidar_rx = !bad_stop; #BIT_NS;
            lidar_rx = 1;
        end
    endtask
    task header;
        input [7:0] status, command;
        input [15:0] length;
        begin
            send_byte(status, 0); send_byte(command, 0);
            send_byte(length[7:0], 0); send_byte(length[15:8], 0);
        end
    endtask
    task configure;
        input ack_payload;
        begin
            expect_request(1);
            header(8'h53, 1, ack_payload ? 1 : 0);
            if (ack_payload) send_byte(0, 0);
            #20000;
            if (initialized !== 1) $fatal(1, "Mode ACK not accepted");
        end
    endtask
    task check_image;
        input integer base;
        integer p;
        begin
            for (p = 0; p < 64; p = p + 1) begin
                read_address = p; #2000;
                if (distance_mm !== base + p)
                    $fatal(1, "Pixel %0d: got %0d expected %0d", p, distance_mm, base+p);
            end
        end
    endtask
    task send_pixels;
        input integer base, count;
        integer p;
        reg [15:0] pixel;
        begin
            for (p = 0; p < count; p = p + 1) begin
                pixel = base + p;
                send_byte(pixel[7:0], 0); send_byte(pixel[15:8], 0);
            end
        end
    endtask
    task check_failure;
        begin
            #1200000;
            if (errors != old_errors + 1 || initialized !== 0 ||
                image_valid !== 1 || frame_count !== 2)
                $fatal(1, "Error recovery/status or damaged publication");
            check_image(2000);
        end
    endtask
    initial begin
        $dumpfile("LidarArray.vcd");
        // Avoid recording all clocks/internal counters during timeout tests.
        $dumpvars(0, rst, lidar_rx, lidar_tx, read_address, distance_mm,
                  image_valid, frame_valid, frame_count, initialized, error);
        repeat (3) @(negedge clk);
        if (image_valid !== 0 || distance_mm !== 0 || frame_count !== 0)
            $fatal(1, "Initial image reset contract");
        rst = 0;
        configure(0);
        expect_request(0);
        header(8'h53, 2, 128);
        send_pixels(1000, 64);
        #20000;
        if (frame_count !== 1 || image_valid !== 1 || frames != 1 || errors != 0)
            $fatal(1, "First image not published");
        check_image(1000);

        expect_request(0);
        header(8'h53, 2, 128);
        send_pixels(2000, 32);
        check_image(1000); // Partial next scan must not change any visible pixel.
        for (n = 32; n < 64; n = n + 1) begin
            send_byte((2000+n) & 255, 0); send_byte((2000+n) >> 8, 0);
        end
        #20000;
        if (frame_count !== 2 || frames != 2) $fatal(1, "Second frame publication");
        check_image(2000);

        // Truncated payload, including an unpaired low byte.
        expect_request(0); old_errors = errors;
        header(8'h53, 2, 128); send_pixels(3000, 5); send_byte(8'h53, 0);
        check_failure;
        configure(1); // Also exercise a mode ACK carrying a payload byte.

        // Incorrect high length byte must not be ignored.
        expect_request(0); old_errors = errors;
        header(8'h53, 2, 16'h0180);
        // Header-like garbage in a rejected response must be drained.
        header(8'h53, 2, 128); send_pixels(900, 2);
        check_failure;
        configure(0);

        expect_request(0); old_errors = errors;
        header(8'h53, 3, 128); // Wrong command.
        check_failure;
        configure(0);

        expect_request(0); old_errors = errors;
        header(8'h63, 2, 1); send_byte(7, 0); // Device reports failure.
        check_failure;
        configure(0);

        expect_request(0); old_errors = errors;
        header(8'h53, 2, 128); send_byte(8'h12, 1); // Framing error.
        check_failure;
        configure(0);

        expect_request(0); old_errors = errors;
        #30500000; // Sensor never replies: total response deadline.
        if (errors != old_errors + 1 || frame_count !== 2 || initialized !== 0)
            $fatal(1, "No-response timeout");
        check_image(2000);
        configure(0);

        expect_request(0);
        header(8'h53, 2, 128); send_pixels(3500, 64);
        #20000;
        if (frame_count !== 3 || frames != 3) $fatal(1, "No recovery after faults");
        check_image(3500);

        expect_request(0);
        header(8'h53, 2, 128); send_pixels(1500, 4);
        @(negedge clk); rst = 1;
        repeat (3) @(negedge clk);
        if (image_valid !== 0 || frame_count !== 0 || distance_mm !== 0 ||
            initialized !== 0 || lidar_tx !== 1 || error !== 0 || frame_valid !== 0)
            $fatal(1, "Reset during scan");
        rst = 0;
        configure(0);
        expect_request(0);
        header(8'h53, 2, 128); send_pixels(4000, 64);
        #20000;
        if (frame_count !== 1 || image_valid !== 1) $fatal(1, "Recovery after reset");
        check_image(4000); // Preserve raw values, including firmware invalid marker.
        $display("PASS: LidarArray commands, 64 pixels, atomic publication, errors, timeouts, reset, Drone routing");
        $finish;
    end
    initial begin #400000000; $fatal(1, "Lidar watchdog"); end
endmodule
