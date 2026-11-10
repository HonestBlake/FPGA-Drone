`timescale 1ns / 1ps
`include "Icm20948Model.vh"
`timescale 1ns / 1ps
module FlightClock_tb;
    reg clk=0, tx=1;
    wire cs,sclk,mosi,miso;
    wire [3:0] esc;
    wire [15:0] led;
    time last_core=0, last_tick=0, began;
    integer ticks=0, bit_number, i, b, sum;
    reg [7:0] packet [0:31];
    always #5 clk=~clk;
    FlightTest #(.IMU_POWERUP_CYCLES(20), .IMU_RESET_CYCLES(20),
        .IMU_WAKE_CYCLES(20), .IMU_CALIBRATION_SAMPLES(4), .MOTOR_STARTUP_MS(1)) dut (
        .clk(clk), .btnC(1'b0), .tx(tx), .sw(5'b01111),
        .imu_spi_cs_n(cs), .imu_spi_sclk(sclk), .imu_spi_mosi(mosi), .imu_spi_miso(miso),
        .esc(esc), .seg(), .dp(), .an(), .led(led));
    Icm20948Model sensor(dut.rst,cs,sclk,mosi,miso);
    always @(posedge dut.logic_clk) begin
        if(last_core != 0 && $time-last_core != 100) $fatal(1,"Core clock not 10 MHz");
        last_core=$time;
    end
    always @(posedge dut.control_tick) begin
        if(last_tick != 0 && $time-last_tick != 1_000_000) $fatal(1,"Control tick not 1 kHz");
        last_tick=$time;
        ticks=ticks+1;
    end
    initial begin
        wait(!dut.rst);
        // Every DShot bit in the stopped packet must have the rounded 10 MHz
        // timing: 1.2 us high in a 3.3 us bit (ordinary DShot300 stop frame).
        @(posedge dut.motors.send); @(posedge dut.logic_clk); #1;
        began=$time-1;
        for(bit_number=0;bit_number<16;bit_number=bit_number+1) begin
            @(negedge esc[0]);
            if($time-began != 1200) $fatal(1,"DShot300 stop pulse timing");
            if(bit_number != 15) begin @(posedge esc[0]); began=$time; end
        end
        wait(led[10]); #1;
        if(!led[11] || led[12] || ticks < 7 || led[4]) $fatal(1,"Board-clock initialization/calibration status");
        packet[0]=8'h20; packet[1]=8'h40;
        for(i=1;i<=14;i=i+1) begin packet[i*2]=8'hdc; packet[i*2+1]=8'h05; end
        packet[6]=8'hc6; packet[7]=8'h06; // Throttle 1734 -> 734.
        packet[10]=8'he8; packet[11]=8'h03; // Arm OFF.
        sum=65535;
        for(i=0;i<30;i=i+1) sum=sum-packet[i];
        packet[30]=sum & 255; packet[31]=(sum>>8) & 255;
        for(i=0;i<32;i=i+1) begin
            tx=0; #8681;
            for(b=0;b<8;b=b+1) begin tx=packet[i][b]; #8681; end
            tx=1; #8681;
        end
        #5000;
        if(!led[6] || dut.throttle != 734 || led[4]) $fatal(1,"115200 baud radio on actual divided board clock");
        $display("PASS: 100 MHz board -> 10 MHz core, real 1 kHz control, DShot300 timing, physical SPI calibration, 115200 baud radio.");
        $finish;
    end
    initial begin #20_000_000; $fatal(1,"FlightClock timeout"); end
endmodule
