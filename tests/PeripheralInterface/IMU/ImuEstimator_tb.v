`include "../../../Drone/PeripheralInterface/IMU/ImuEstimator.v"
`timescale 1ns / 1ps
module ImuEstimator_tb;
    reg clk=0, rst=1, raw_valid=0, ready=1, sensor_fault=0;
    reg recover_request=0;
    wire [3:0] fault_reason;
    always #50 clk=~clk;
    reg signed [15:0] ax=0, ay=0, az=16384, gx=131, gy=-131, gz=65;
    wire sample_valid, calibrated, fault, valid;
    wire signed [15:0] roll, pitch, rx, ry, rz;
    integer sample_spacing=20000;
    ImuEstimator #(.CLK_FREQ(10_000_000), .CALIBRATION_SAMPLES(4), .TIMEOUT_MS(10),
        .RECOVERY_ENABLED(1)) dut (
        .clk(clk), .rst(rst), .raw_valid(raw_valid), .sensor_ready(ready), .sensor_fault(sensor_fault),
        .accel_x(ax), .accel_y(ay), .accel_z(az), .gyro_x(gx), .gyro_y(gy), .gyro_z(gz),
        .sample_valid(sample_valid), .calibrated(calibrated), .fault(fault), .valid(valid),
        .recover_request(recover_request), .fault_reason(fault_reason),
        .roll_cd(roll), .pitch_cd(pitch), .roll_rate_cdps(rx), .pitch_rate_cdps(ry), .yaw_rate_cdps(rz));
    task restart;
        begin
            @(negedge clk); rst=1; raw_valid=0;
            repeat(3) @(negedge clk); rst=0;
        end
    endtask
    task sample;
        begin
            repeat(sample_spacing) @(negedge clk); raw_valid=1;
            @(negedge clk); raw_valid=0;
        end
    endtask
    initial begin
        $dumpfile("ImuEstimator.vcd"); $dumpvars(0,clk,rst,raw_valid,valid,calibrated,fault,roll,pitch,rx,ry,rz);
        restart;
        sample; sample;
        ax=2000; sample;
        if(calibrated || dut.count != 0) $fatal(1,"Moving startup calibrated");
        ax=0;
        sample; sample; sample;
        if(calibrated || valid) $fatal(1,"Calibration finished too early");
        sample;
        if(!calibrated || !valid || !sample_valid || roll != 0 || pitch != 0)
            $fatal(1,"Level calibration failed");
        sample;
        if(rx != 0 || ry != 0 || rz != 0) $fatal(1,"Gyro bias cancellation failed");
        gx=1441; gy=-1441; gz=-1245;
        sample;
        if(rx != 1000 || ry != 1000 || rz != 1000 || roll < 1 || pitch < 1)
            $fatal(1,"Positive rate scale, signs or integration failed: %d %d %d %d %d",rx,ry,rz,roll,pitch);
        gx=-1179; gy=1179; gz=1375;
        sample;
        if(rx != -1000 || ry != -1000 || rz != -1000)
            $fatal(1,"Negative rate scale or signs failed");
        // Gravity correction: 10 deg tilt has sin(theta)*16384 = 2845.
        gx=131; gy=-131; gz=65; ay=2845; ax=-2845; az=15890;
        sample_spacing=100;
        repeat(800) sample;
        if(roll < 945 || roll > 995 || pitch > -945 || pitch < -995)
            $fatal(1,"Gravity correction signs/convergence: %d %d",roll,pitch);
        // Translation outside the gravity gate must not tilt the estimate.
        ax=16000; ay=0; az=16384;
        sample;
        if(roll < 945 || pitch > -945) $fatal(1,"Dynamic acceleration used as gravity");
        #10_100_000;
        if(!fault || valid || calibrated || fault_reason != 1) $fatal(1,"Stale sample not faulted");
        ax=0; ay=0; az=16384; restart;
        repeat(4) sample;
        gx=-32768; sample;
        if(!fault || valid || sample_valid || fault_reason != 2) $fatal(1,"Saturation accepted");
        gx=131; restart; repeat(4) sample;
        sensor_fault=1; @(negedge clk);
        if(!fault || valid) $fatal(1,"Bus fault ignored");
        sensor_fault=0; sample;
        if(!fault || calibrated) $fatal(1,"Fault recovered without reset");
        recover_request=1;
        sensor_fault=1; sample;
        if(!fault || fault_reason != 6) $fatal(1,"Active bus failure recovered");
        sensor_fault=0; ax=2000; sample;
        if(!fault || valid) $fatal(1,"Moving data cleared a fault");
        ax=0; sample;
        if(fault || calibrated || valid || sample_valid || fault_reason != 0)
            $fatal(1,"Recovery did not require fresh calibration");
        repeat(3) sample;
        if(valid || calibrated) $fatal(1,"Recovery calibration completed early");
        sample;
        if(!valid || !calibrated || fault) $fatal(1,"Healthy recovery failed");
        az=32767; sample;
        if(!fault || valid || fault_reason != 3) $fatal(1,"Accel clipping reason failed");
        repeat(3) sample;
        if(!fault || valid) $fatal(1,"Persistent saturation auto-cleared");
        az=16384; recover_request=0; repeat(4) sample;
        if(!fault || valid || fault_reason != 3) $fatal(1,"Recovery ignored parent permission");
        recover_request=1; sample; repeat(4) sample;
        if(!valid) $fatal(1,"Accel clipping recovery failed");
        recover_request=0; gx=30000; sample_spacing=90000;
        repeat(24) sample;
        if(!fault || fault_reason != 4) $fatal(1,"Tilt fault reason failed");
        gx=131; sample_spacing=100; ready=0; recover_request=1; sample;
        if(!fault || valid) $fatal(1,"Not-ready sensor recovered");
        ready=1; sample; repeat(4) sample;
        if(!valid || fault) $fatal(1,"Tilt recovery failed");
        $display("PASS: ImuEstimator stationary calibration, bias, signs, units, elapsed time, tilt, acceleration rejection and faults");
        $finish;
    end
    initial begin #2_000_000_000; $fatal(1,"ImuEstimator timeout"); end
endmodule
