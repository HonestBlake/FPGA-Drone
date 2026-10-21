`include "../../../Drone/PeripheralInterface/IMU/imu_spi.v"
`include "../../support/Icm20948Model.vh"
`timescale 1ns / 1ps
module imu_spi_tb;
    reg clk=0, reset=1, start=0;
    always #50 clk=~clk;
    reg board_clk=0;
    always #5 board_clk=~board_clk;
    wire cs, sclk, mosi, miso, data_valid, busy, init_done, detected, error;
    wire signed [15:0] ax, ay, az, gx, gy, gz;
    imu_spi #(.CLK_FREQ(10_000_000), .CLK_DIV(1), .POWERUP_CYCLES(20),
        .RESET_CYCLES(20), .WAKE_CYCLES(20)) dut (
        .clk(clk), .reset(reset), .start(start), .spi_cs_n(cs), .spi_sclk(sclk),
        .spi_mosi(mosi), .spi_miso(miso), .accel_x(ax), .accel_y(ay), .accel_z(az),
        .gyro_x(gx), .gyro_y(gy), .gyro_z(gz), .data_valid(data_valid), .busy(busy),
        .init_done(init_done), .imu_detected(detected), .error(error));
    Icm20948Model sensor(reset,cs,sclk,mosi,miso);
    // Exercise the production 100 MHz / default-divider SPI edge timing too.
    wire board_cs,board_sclk,board_mosi,board_miso,board_init,board_error;
    imu_spi #(.POWERUP_CYCLES(20), .RESET_CYCLES(20), .WAKE_CYCLES(20)) board_driver (
        .clk(board_clk), .reset(reset), .start(1'b0), .spi_cs_n(board_cs), .spi_sclk(board_sclk),
        .spi_mosi(board_mosi), .spi_miso(board_miso), .init_done(board_init), .error(board_error));
    Icm20948Model board_sensor(reset,board_cs,board_sclk,board_mosi,board_miso);
    task restart;
        begin
            @(negedge clk); reset=1; start=0;
            repeat(3) @(negedge clk); reset=0;
        end
    endtask
    task request;
        begin
            wait(!busy); @(negedge clk); start=1;
            @(negedge clk); start=0;
        end
    endtask
    initial begin
        $dumpfile("imu_spi.vcd"); $dumpvars(0,imu_spi_tb);
        restart; wait(init_done); #1;
        if(!board_init || board_error) $fatal(1,"100 MHz default SPI initialization/timing failed");
        if (!detected || error || sensor.resets != 1 || sensor.bank != 0 ||
            sensor.registers[3] != 8'h10 || sensor.registers[6] != 1 || sensor.registers[7] != 0 ||
            sensor.registers[256] != 1 || sensor.registers[257] != 8'h19 ||
            sensor.registers[272] != 0 || sensor.registers[273] != 1 || sensor.registers[276] != 8'h19)
            $fatal(1,"Initialization, bank recovery or configuration failed");
        sensor.ax=4096; sensor.ay=8192; sensor.az=16384;
        sensor.gx=256; sensor.gy=512; sensor.gz=768;
        request; wait(data_valid); #1;
        if ({ax,ay,az,gx,gy,gz} !== {16'd4096,16'd8192,16'd16384,16'd256,16'd512,16'd768})
            $fatal(1,"Burst sample failed");
        repeat(2) @(negedge clk);
        if(data_valid) $fatal(1,"data_valid not one clock");
        sensor.ax=-1; sensor.ay=-4096; sensor.az=4096;
        sensor.gx=-256; sensor.gy=512; sensor.gz=-32768;
        request; wait(data_valid); #1;
        if ({ax,ay,az,gx,gy,gz} !== {16'hffff,16'hf000,16'h1000,16'hff00,16'h0200,16'h8000})
            $fatal(1,"Signed burst failed");
        sensor.ready_enable=0; request; wait(!busy); #1;
        if(data_valid || sensor.reads != 2) $fatal(1,"Stale registers published as fresh");
        sensor.disconnected=1; request; wait(error); #1;
        if(init_done || detected || data_valid) $fatal(1,"Disconnect health failed");
        sensor.disconnected=0; sensor.ignore_config=1; restart; wait(error);
        if(init_done) $fatal(1,"Bad configuration readback accepted");
        sensor.ignore_config=0; sensor.who=0; restart; wait(error);
        sensor.who=8'hea; sensor.ready_enable=1; restart; wait(init_done);
        request; wait(!cs); repeat(10) @(negedge clk);
        restart; wait(init_done); request; wait(data_valid);
        $display("PASS: imu_spi bank/reset recovery, timing, config readback, signed bursts, freshness and disconnect");
        $finish;
    end
    initial begin #3_000_000; $fatal(1,"imu_spi timeout"); end
endmodule
