`include "../../../Drone/System/TickGenerator.v"
`include "../../../Drone/PeripheralInterface/MotorControl/MotorDriver.v"
`include "../../../Drone/PeripheralInterface/MotorControl/MotorControl.v"
`timescale 1ns / 1ps
module MotorControl_tb;
    reg clk = 0, rst = 1, armed = 1, valid = 1;
    reg [10:0] fr = 48, fl = 1000, br = 2047, bl = 1048;
    wire [3:0] esc;
    wire ready, busy;
    integer frame, bit_number, cycle, channel;
    integer high_count [0:3];
    reg [15:0] decoded [0:3];
    time last_start = 0;
    always #5 clk = ~clk;
    // 40 clocks/bit, 1,000 clocks/update, 12 startup stop frames.
    MotorControl #(.CLK_FREQ(12_000_000), .UPDATE_HZ(12_000), .STARTUP_MS(1)) dut (
        .clk(clk), .rst(rst), .motors_armed(armed), .command_valid(valid),
        .front_right_throttle(fr), .front_left_throttle(fl),
        .back_right_throttle(br), .back_left_throttle(bl),
        .esc_front_right(esc[0]), .esc_front_left(esc[1]),
        .esc_back_right(esc[2]), .esc_back_left(esc[3]), .ready(ready), .busy(busy)
    );
    task check_frame;
        input [63:0] expected;
        begin
            @(posedge esc[0]); #1;
            if (last_start != 0 && $time - last_start != 10_000)
                $fatal(1, "Wrong motor update interval");
            last_start = $time;
            if (esc !== 4'b1111 || !busy) $fatal(1, "Motor starts are not synchronized");
            for (channel = 0; channel < 4; channel = channel + 1) decoded[channel] = 0;
            for (bit_number = 15; bit_number >= 0; bit_number = bit_number - 1) begin
                for (channel = 0; channel < 4; channel = channel + 1) high_count[channel] = 0;
                for (cycle = 0; cycle < 40; cycle = cycle + 1) begin
                    for (channel = 0; channel < 4; channel = channel + 1) begin
                        if (esc[channel] === 1'b1) high_count[channel] = high_count[channel] + 1;
                        else if (esc[channel] !== 1'b0) $fatal(1, "Unknown motor output");
                    end
                    @(posedge clk); #1;
                end
                for (channel = 0; channel < 4; channel = channel + 1) begin
                    if (high_count[channel] == 30) decoded[channel][bit_number] = 1;
                    else if (high_count[channel] != 15) $fatal(1, "Wrong motor pulse width");
                end
            end
            if ({decoded[3], decoded[2], decoded[1], decoded[0]} !== expected)
                $fatal(1, "Motor packets %h %h %h %h, expected %h",
                    decoded[3], decoded[2], decoded[1], decoded[0], expected);
            // Check low gap and all idle clocks up to just before the next update.
            for (cycle = 0; cycle < 359; cycle = cycle + 1) begin
                if (esc !== 4'b0000) $fatal(1, "Motor line high between packets");
                if (busy !== (cycle < 80)) $fatal(1, "Wrong motor busy/gap interval");
                @(posedge clk); #1;
            end
        end
    endtask
    initial begin
        $dumpfile("MotorControl.vcd"); $dumpvars(0, MotorControl_tb);
        repeat (2) @(negedge clk);
        if (esc !== 0 || ready !== 0 || busy !== 0) $fatal(1, "Motor reset failed");
        rst = 0;
        for (frame = 0; frame < 12; frame = frame + 1) begin
            if (ready) $fatal(1, "Startup interval too short");
            check_frame(64'd0);
        end
        if (!ready) $fatal(1, "Startup did not complete");
        check_frame(64'h830b_ffee_7d0a_0606);
        @(negedge clk); armed = 0;
        check_frame(0);
        @(negedge clk); armed = 1; valid = 0;
        check_frame(0);
        @(negedge clk); valid = 1; fr = 1; fl = 47; br = 0;
        check_frame(64'h830b_0000_0000_0000);
        // Reset also restarts the startup stop interval.
        @(negedge clk); rst = 1;
        @(posedge clk); #1;
        if (esc !== 0 || ready || busy) $fatal(1, "Reset did not stop all channels");
        @(negedge clk); rst = 0; last_start = 0;
        check_frame(0);
        if (ready) $fatal(1, "Reset did not restart startup interval");
        $display("PASS: four independent throttles, startup, update rate, disarm, validity and reset");
        $finish;
    end
    initial begin #1_000_000; $fatal(1, "MotorControl test timeout"); end
endmodule
