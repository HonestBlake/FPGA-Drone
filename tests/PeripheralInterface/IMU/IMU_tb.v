`include "../../../Drone/PeripheralInterface/IMU/imu_spi.v"
`include "../../../Drone/PeripheralInterface/IMU/ImuEstimator.v"
`include "../../../Drone/PeripheralInterface/IMU/IMU.v"
`include "../../support/Icm20948Model.vh"
`timescale 1ns / 1ps
module IMU_tb;
    reg clk=0, rst=1;
    always #50 clk=~clk;
    wire cs,sclk,mosi,miso,raw_valid,busy,init_done,detected,sample_valid,valid,calibrated,fault;
    wire signed [15:0] roll,pitch,rx,ry,rz;
    integer samples=0;
    IMU #(.CLK_FREQ(10_000_000), .POWERUP_CYCLES(20), .RESET_CYCLES(20), .WAKE_CYCLES(20),
        .CALIBRATION_SAMPLES(4), .TIMEOUT_MS(10)) dut (
        .clk(clk), .rst(rst), .spi_cs_n(cs), .spi_sclk(sclk), .spi_mosi(mosi), .spi_miso(miso),
        .raw_valid(raw_valid), .busy(busy), .init_done(init_done), .detected(detected),
        .sample_valid(sample_valid), .valid(valid), .calibrated(calibrated), .fault(fault),
        .roll_cd(roll), .pitch_cd(pitch), .roll_rate_cdps(rx), .pitch_rate_cdps(ry), .yaw_rate_cdps(rz));
    Icm20948Model sensor(rst,cs,sclk,mosi,miso);
    always @(negedge clk) if(sample_valid) samples=samples+1;
    task restart;
        begin @(negedge clk); rst=1; repeat(3) @(negedge clk); rst=0; end
    endtask
    initial begin
        $dumpfile("IMU.vcd"); $dumpvars(0,clk,rst,cs,sclk,mosi,miso,raw_valid,init_done,valid,calibrated,fault,roll,pitch,rx,ry,rz);
        restart; wait(init_done);
        if(valid || calibrated) $fatal(1,"Init bypassed calibration");
        wait(calibrated); #1;
        if(!valid || fault || sensor.reads != 4) $fatal(1,"Polling/calibration integration");
        sensor.gx=1310; sensor.gy=1310; sensor.gz=-1310;
        wait(!sample_valid); wait(sample_valid); #1;
        if(rx != 1000 || ry != -1000 || rz != 1000) $fatal(1,"SPI to estimator rates failed");
        sensor.ready_enable=0;
        #12_000_000;
        if(valid || !fault || calibrated) $fatal(1,"No-ready sensor remained fresh");
        sensor.ready_enable=1; sensor.gx=0; sensor.gy=0; sensor.gz=0;
        restart; wait(calibrated); sensor.disconnected=1; wait(fault); #1;
        if(valid || detected || init_done) $fatal(1,"Disconnect integration");
        $display("PASS: IMU physical SPI -> calibration/estimator, polling, stale-ready and disconnect");
        $finish;
    end
    initial begin #50_000_000; $fatal(1,"IMU timeout"); end
endmodule
