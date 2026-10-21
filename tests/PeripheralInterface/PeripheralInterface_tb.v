`include "../../Drone/PeripheralInterface/IMU/imu_spi.v"
`include "../../Drone/PeripheralInterface/IMU/ImuEstimator.v"
`include "../../Drone/PeripheralInterface/IMU/IMU.v"
`include "../../Drone/PeripheralInterface/UartReceiver.v"
`include "../../Drone/PeripheralInterface/RadioReceiver/IBusDecoder.v"
`include "../../Drone/PeripheralInterface/RadioReceiver/RCMapper.v"
`include "../../Drone/PeripheralInterface/RadioReceiver/RadioReceiver.v"
`include "../../Drone/UartTransmitter.v"
`include "../../Drone/PeripheralInterface/LidarArray/LidarArray.v"
`include "../../Drone/PeripheralInterface/PeripheralInterface.v"
`include "../../Drone/System/TickGenerator.v"
`include "../../Drone/PeripheralInterface/MotorControl/MotorDriver.v"
`include "../../Drone/PeripheralInterface/MotorControl/MotorControl.v"
`timescale 1ns / 1ps
module PeripheralInterface_tb;
    `include "../support/RadioInterfaceChecks.vh"
    PeripheralInterface #(
        .CLK_FREQ(10_000_000), .BAUD_RATE(1_000_000),
        .TIMEOUT_MS(1), .BYTE_TIMEOUT_US(50),
        .ROLL_CHANNEL(14), .PITCH_CHANNEL(3), .THROTTLE_CHANNEL(2),
        .YAW_CHANNEL(4), .ARM_CHANNEL(6), .MODE_CHANNEL(5), .SWITCH_THRESHOLD(1750)
    ) dut (
        .clk(clk), .rst(rst), .imu_spi_miso(1'b0), .ibus_rx(ibus_rx), .throttle(throttle),
        .roll_command(roll_command), .pitch_command(pitch_command),
        .yaw_command(yaw_command), .arm_request(arm_request),
        .avoidance_enable(avoidance_enable), .frame_valid(frame_valid),
        .rc_valid(rc_valid), .channels(channels),
        .lidar_rx(1'b1), .lidar_read_address(6'd0),
        .motors_armed(1'b0), .motor_command_valid(1'b0),
        .front_right_throttle(11'd0), .front_left_throttle(11'd0),
        .back_right_throttle(11'd0), .back_left_throttle(11'd0)
    );
    initial begin
        $dumpfile("PeripheralInterface.vcd");
        $dumpvars(0, clk, rst, ibus_rx, throttle, roll_command, pitch_command,
                  yaw_command, arm_request, avoidance_enable, frame_valid, rc_valid, channels);
    end
endmodule
