`include "../Drone/PeripheralInterface/IMU/imu_spi.v"
`include "../Drone/PeripheralInterface/IMU/ImuEstimator.v"
`include "../Drone/PeripheralInterface/IMU/IMU.v"
`include "../Drone/PeripheralInterface/UartReceiver.v"
`include "../Drone/PeripheralInterface/RadioReceiver/IBusDecoder.v"
`include "../Drone/PeripheralInterface/RadioReceiver/RCMapper.v"
`include "../Drone/PeripheralInterface/RadioReceiver/RadioReceiver.v"
`include "../Drone/UartTransmitter.v"
`include "../Drone/PeripheralInterface/LidarArray/LidarArray.v"
`include "../Drone/PeripheralInterface/PeripheralInterface.v"
`include "../Drone/System/TickGenerator.v"
`include "../Drone/System/UartOutput.v"
`include "../Drone/System/SevenSegmentDisplay.v"
`include "../Drone/System/System.v"
`include "../Drone/NPU/NPU.v"
`include "../Drone/FlightController/FlightController.v"
`include "../Drone/PeripheralInterface/MotorControl/MotorDriver.v"
`include "../Drone/PeripheralInterface/MotorControl/MotorControl.v"
`include "../Drone/Drone.v"
`include "support/Icm20948Model.vh"
`timescale 1ns / 1ps
module Drone_tb;
    `define DRONE_FLIGHT_TEST
    `include "support/RadioInterfaceChecks.vh"
    `undef DRONE_FLIGHT_TEST
    `include "support/SystemOutputChecks.vh"
    wire control_tick, npu_available, flight_controller_available, motors_armed;
    wire [3:0] esc;
    wire motor_ready, motor_busy;
    reg physical_rst=1, check_physical=0;
    wire physical_cs, physical_sclk, physical_mosi, physical_miso;
    wire physical_calibrated, physical_valid, physical_fault, physical_armed, physical_command_valid;
    wire physical_busy;
    wire [3:0] physical_esc;
    wire [10:0] physical_fr, physical_fl, physical_br, physical_bl;
    wire [3:0] checked_esc = check_physical ? physical_esc : esc;
    wire checked_busy = check_physical ? physical_busy : motor_busy;
    // Default hardware selection: this instance receives sensor bytes at SPI
    // pins and real iBUS packets, with no external attitude/health inputs.
    Drone #(.CLK_FREQ(10_000_000), .BAUD_RATE(1_000_000), .TIMEOUT_MS(100),
        .BYTE_TIMEOUT_US(50), .ROLL_CHANNEL(14), .PITCH_CHANNEL(3), .THROTTLE_CHANNEL(2),
        .YAW_CHANNEL(4), .ARM_CHANNEL(6), .MODE_CHANNEL(5), .SWITCH_THRESHOLD(1750),
        .IMU_POWERUP_CYCLES(20), .IMU_RESET_CYCLES(20), .IMU_WAKE_CYCLES(20),
        .IMU_CALIBRATION_SAMPLES(2), .IMU_TIMEOUT_MS(10),
        .MOTOR_STARTUP_MS(1), .MOTOR_UPDATE_HZ(10_000)) physical (
        .clk(clk), .rst(physical_rst), .ibus_rx(ibus_rx), .imu_spi_miso(physical_miso),
        .imu_spi_cs_n(physical_cs), .imu_spi_sclk(physical_sclk), .imu_spi_mosi(physical_mosi),
        .imu_sensor_calibrated(physical_calibrated), .imu_sensor_valid(physical_valid),
        .imu_sensor_fault(physical_fault), .motors_armed(physical_armed),
        .flight_command_valid(physical_command_valid),
        .front_right_throttle(physical_fr), .front_left_throttle(physical_fl),
        .back_right_throttle(physical_br), .back_left_throttle(physical_bl),
        .esc_front_right(physical_esc[0]), .esc_front_left(physical_esc[1]),
        .esc_back_right(physical_esc[2]), .esc_back_left(physical_esc[3]), .motor_busy(physical_busy),
        .lidar_rx(1'b1), .lidar_read_address(6'd0), .uart_data(8'd0), .uart_valid(1'b0),
        .display_write(1'b0), .display_value(16'd0), .display_digit_enable(4'd0), .display_decimal_points(4'd0),
        .imu_sample_valid(1'b0), .imu_valid(1'b0), .imu_calibrated(1'b0), .imu_fault(1'b1),
        .imu_roll_cd(16'sd0), .imu_pitch_cd(16'sd0), .imu_roll_rate_cdps(16'sd0),
        .imu_pitch_rate_cdps(16'sd0), .imu_yaw_rate_cdps(16'sd0),
        .npu_external_available(1'b0), .npu_external_action_valid(1'b0), .npu_external_action(3'd0));
    Icm20948Model physical_sensor(physical_rst,physical_cs,physical_sclk,physical_mosi,physical_miso);
    integer tick_phase = 0;
    reg flight_phase = 0, imu_stream = 0, imu_health = 0, imu_fault = 0;
    reg npu_external_available = 0, npu_external_action_valid = 0;
    reg [2:0] npu_external_action = 0;
    wire imu_fresh, flight_command_valid;
    integer flight_cycle = 0;
    wire imu_sample_valid = imu_stream && flight_cycle % 100 == 0;
    always @(negedge clk) flight_cycle = flight_cycle + 1;
    Drone #(
        .CLK_FREQ(10_000_000), .BAUD_RATE(1_000_000),
        .TIMEOUT_MS(1), .BYTE_TIMEOUT_US(50),
        .ROLL_CHANNEL(14), .PITCH_CHANNEL(3), .THROTTLE_CHANNEL(2),
        .YAW_CHANNEL(4), .ARM_CHANNEL(6), .MODE_CHANNEL(5), .SWITCH_THRESHOLD(1750),
        .CONTROL_TICK_FREQ(1_000_000), .UART_BAUD_RATE(1_000_000), .UART_FIFO_DEPTH(2),
        .DISPLAY_REFRESH_HZ(25_000), .DISPLAY_BLANK_CYCLES(3),
        .MOTOR_UPDATE_HZ(10_000), .MOTOR_STARTUP_MS(1), .USE_EXTERNAL_NPU(1), .USE_EXTERNAL_IMU(1)
    ) dut (
        .clk(clk), .rst(rst), .imu_spi_miso(1'b0), .ibus_rx(ibus_rx), .throttle(throttle),
        .roll_command(roll_command), .pitch_command(pitch_command),
        .yaw_command(yaw_command), .arm_request(arm_request),
        .avoidance_enable(avoidance_enable), .frame_valid(frame_valid),
        .rc_valid(rc_valid), .channels(channels),
        .control_tick(control_tick), .npu_available(npu_available),
        .flight_controller_available(flight_controller_available), .motors_armed(motors_armed),
        .imu_sample_valid(imu_sample_valid), .imu_valid(imu_health),
        .imu_calibrated(imu_health), .imu_fault(imu_fault),
        .imu_roll_cd(16'sd0), .imu_pitch_cd(16'sd0),
        .imu_roll_rate_cdps(16'sd0), .imu_pitch_rate_cdps(16'sd0), .imu_yaw_rate_cdps(16'sd0),
        .npu_external_available(npu_external_available),
        .npu_external_action_valid(npu_external_action_valid), .npu_external_action(npu_external_action),
        .imu_fresh(imu_fresh), .flight_command_valid(flight_command_valid),
        .lidar_rx(1'b1), .lidar_read_address(6'd0),
        .uart_data(uart_data), .uart_valid(uart_valid), .uart_ready(uart_ready),
        .uart_busy(uart_busy), .uart_tx(uart_tx), .display_write(display_write),
        .display_value(display_value), .display_digit_enable(display_digit_enable),
        .display_decimal_points(display_decimal_points), .seg(seg), .dp(dp), .an(an),
        .esc_front_right(esc[0]), .esc_front_left(esc[1]),
        .esc_back_right(esc[2]), .esc_back_left(esc[3]),
        .motor_ready(motor_ready), .motor_busy(motor_busy)
    );
    initial begin
        $dumpfile("Drone.vcd");
        $dumpvars(0, clk, rst, ibus_rx, throttle, roll_command, pitch_command,
                  yaw_command, arm_request, avoidance_enable, frame_valid, rc_valid,
                  control_tick, npu_available, flight_controller_available, motors_armed);
    end
    always @(posedge clk) begin
        #2;
        if (rst) tick_phase = 0;
        else tick_phase = tick_phase + 1;
        if (control_tick !== (!rst && tick_phase % 10 == 0))
            $fatal(1, "Drone system timing or reset wiring failed");
        if ({npu_available, flight_controller_available} !== {npu_external_available, !rst})
            $fatal(1, "Drone controller or selected NPU availability incorrect");
        if (!flight_phase && (motors_armed || {dut.front_right_throttle, dut.front_left_throttle,
             dut.back_right_throttle, dut.back_left_throttle} !== 44'd0 ||
            dut.peripheral_interface.motor_control.command_valid !== 1'b0))
            $fatal(1, "Drone motor command routing failed");
        if (dut.peripheral_interface.motor_control.command_valid !== (rc_valid && flight_command_valid))
            $fatal(1, "Drone command health not routed to ESCs");
        if (motors_armed && (!imu_fresh || !rc_valid || !motor_ready || !flight_command_valid))
            $fatal(1, "Drone armed with invalid input");
        if ((!flight_phase && esc !== {4{esc[0]}}) || (rst && (esc !== 0 || motor_busy || motor_ready)))
            $fatal(1, "Drone motor outputs or reset incorrect");
        if ({dut.flight_controller.throttle, dut.flight_controller.roll_command,
             dut.flight_controller.pitch_command, dut.flight_controller.yaw_command,
             dut.flight_controller.arm_request, dut.flight_controller.avoidance_enable,
             dut.flight_controller.rc_valid, dut.flight_controller.frame_valid,
             dut.flight_controller.control_tick, dut.flight_controller.npu_available} !==
            {throttle, roll_command, pitch_command, yaw_command, arm_request,
             avoidance_enable, rc_valid, frame_valid, control_tick, npu_available})
            $fatal(1, "Drone flight-controller input routing failed");
    end

    // Continue the shared radio integration checks into actual flight commands.
    task send_controls;
        input integer thrust, forward;
        input arm, avoid;
        integer k, checksum;
        begin
            set_channel(2, 1000 + thrust); set_channel(3, 1500 + forward);
            set_channel(4, 1500); set_channel(6, arm ? 2000 : 1000);
            set_channel(5, avoid ? 2000 : 1000);
            checksum = 0;
            for (k = 0; k < 30; k = k + 1) checksum = checksum + packet[k];
            checksum = 65535 - checksum;
            packet[30] = checksum & 255; packet[31] = (checksum >> 8) & 255;
            for (k = 0; k < 28; k = k + 1) expected_channels[k*8 +: 8] = packet[k+2];
            send_frame;
        end
    endtask

    // Decode all four real ESC waveforms at 10 MHz / DShot300 (33 clocks/bit).
    task check_esc_frame;
        input [63:0] expected;
        integer bit_index, clock_index, channel;
        integer highs [0:3];
        reg [15:0] decoded [0:3];
        begin
            wait (!checked_busy);
            @(posedge checked_esc[0]); #2;
            if (checked_esc !== 4'b1111) $fatal(1, "Drone ESC packet starts differ");
            for (channel = 0; channel < 4; channel = channel + 1) decoded[channel] = 0;
            for (bit_index = 15; bit_index >= 0; bit_index = bit_index - 1) begin
                for (channel = 0; channel < 4; channel = channel + 1) highs[channel] = 0;
                for (clock_index = 0; clock_index < 33; clock_index = clock_index + 1) begin
                    for (channel = 0; channel < 4; channel = channel + 1)
                        if (checked_esc[channel] === 1'b1) highs[channel] = highs[channel] + 1;
                        else if (checked_esc[channel] !== 1'b0) $fatal(1, "Drone unknown ESC output");
                    @(posedge clk); #2;
                end
                for (channel = 0; channel < 4; channel = channel + 1)
                    if (highs[channel] == 25) decoded[channel][bit_index] = 1;
                    else if (highs[channel] != 12)
                        $fatal(1, "Drone ESC pulse width: channel %0d bit %0d high %0d", channel, bit_index, highs[channel]);
            end
            if ({decoded[3], decoded[2], decoded[1], decoded[0]} !== expected)
                $fatal(1, "Radio to ESC packets %h %h %h %h, expected %h",
                    decoded[3], decoded[2], decoded[1], decoded[0], expected);
        end
    endtask

    task run_flight_checks;
        begin
            flight_phase = 1; imu_health = 1; imu_stream = 1;
            #20000;
            if (!motor_ready || !flight_command_valid || motors_armed)
                $fatal(1, "Drone readiness or boot arm interlock");
            send_controls(0, 0, 0, 0);
            send_controls(0, 0, 1, 0);
            if (!motors_armed) $fatal(1, "Radio arm did not reach controller");
            send_controls(500, 250, 1, 1);
            if ({dut.front_right_throttle, dut.front_left_throttle,
                 dut.back_right_throttle, dut.back_left_throttle} !== {11'd922, 11'd922, 11'd1172, 11'd1172})
                $fatal(1, "Radio forward/thrust did not reach mixer");
            // Independently calculated DShot payload and XOR checksums.
            check_esc_frame(64'h9283_9283_7340_7340);
            @(negedge clk); npu_external_available = 1;
            npu_external_action = 1; npu_external_action_valid = 1;
            @(negedge clk); npu_external_action_valid = 0;
            #2000;
            if (!dut.avoidance_active || dut.pitch_setpoint_cd !== 0 || !motors_armed ||
                {dut.front_right_throttle, dut.front_left_throttle,
                 dut.back_right_throttle, dut.back_left_throttle} !== {4{11'd1047}})
                $fatal(1, "External NPU STOP integration");
            check_esc_frame(64'h82e4_82e4_82e4_82e4);
            @(negedge clk); imu_fault = 1;
            @(negedge clk);
            if (motors_armed || flight_command_valid) $fatal(1, "IMU fault integration");
            check_esc_frame(0);
            @(negedge clk); imu_fault = 0;
            #20000;
            if (motors_armed) $fatal(1, "Automatic arm after IMU recovery");
            send_controls(0, 0, 0, 0); send_controls(0, 0, 1, 0);
            send_controls(500, 0, 1, 0);
            if (!motors_armed) $fatal(1, "Explicit rearm after IMU recovery");
            #1010000;
            if (rc_valid || motors_armed || flight_command_valid) $fatal(1, "RC loss integration");
            check_esc_frame(0);
            npu_external_available = 0;
            $display("PASS: Drone radio -> stabilization/mixer -> decoded ESC packets, NPU STOP and faults");
            run_physical_imu_checks;
        end
    endtask

    task run_physical_imu_checks;
        begin
            check_physical=1;
            @(negedge clk); physical_rst=0;
            send_controls(0,0,0,0); send_controls(0,0,1,0);
            if(physical_armed || physical_command_valid) $fatal(1,"Physical IMU armed before calibration");
            wait(physical_calibrated); #1000;
            if(!physical_valid || physical_fault || physical_armed)
                $fatal(1,"Physical IMU readiness or boot arm interlock");
            send_controls(0,0,0,0); send_controls(0,0,1,0); send_controls(500,0,1,0);
            #2000000;
            if(!physical_armed || {physical_fr,physical_fl,physical_br,physical_bl} !== {4{11'd1047}})
                $fatal(1,"Physical IMU -> controller -> motor mixer failed");
            check_esc_frame(64'h82e4_82e4_82e4_82e4);
            physical_sensor.gx=1310;
            wait(physical.imu_sensor_roll_rate_cdps == 1000); #1100000;
            if(physical_fr <= physical_fl || physical_br <= physical_bl || !physical_armed)
                $fatal(1,"Measured gyro did not change stabilization/motor outputs");
            physical_sensor.ready_enable=0;
            wait(physical_fault); #2000;
            if(physical_armed || physical_command_valid || {physical_fr,physical_fl,physical_br,physical_bl} !== 0)
                $fatal(1,"Sensor not ready did not stop flight commands");
            check_esc_frame(0);
            // Reset, recalibrate, and verify the arm switch must be cycled.
            @(negedge clk); physical_rst=1;
            physical_sensor.ready_enable=1; physical_sensor.gx=0;
            repeat(3) @(negedge clk); physical_rst=0;
            send_controls(0,0,1,0); wait(physical_calibrated); #1000;
            if(physical_armed) $fatal(1,"Physical IMU automatically armed after recovery");
            send_controls(0,0,0,0); send_controls(0,0,1,0); send_controls(500,0,1,0);
            #1100000;
            if(!physical_armed) $fatal(1,"Physical IMU explicit rearm failed");
            physical_sensor.disconnected=1; wait(physical_fault); #2000;
            if(physical_armed || physical_command_valid) $fatal(1,"Disconnected physical IMU stayed armed");
            check_esc_frame(0);
            $display("PASS: Drone physical SPI IMU -> estimator -> flight controller -> decoded ESC packets, gyro stabilization, stale data, reset/rearm and disconnect");
        end
    endtask
endmodule
