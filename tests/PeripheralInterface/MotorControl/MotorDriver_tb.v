`include "../../../Drone/PeripheralInterface/MotorControl/MotorDriver.v"
`timescale 1ns / 1ps

module MotorDriverChecks #(parameter integer RATE = 300_000) (output reg done = 0);
    localparam integer BITS = (100_000_000 + RATE / 2) / RATE;
    localparam integer H0 = (BITS * 3 + 4) / 8;
    localparam integer H1 = (BITS * 6 + 4) / 8;
    reg clk = 0, rst = 1, send = 0, enable = 1;
    reg [10:0] throttle = 0;
    wire signal_out, busy;
    integer bit_number, cycle, high_count;
    reg [15:0] decoded;
    always #5 clk = ~clk;
    MotorDriver #(.DSHOT_RATE(RATE)) dut (
        .clk(clk), .rst(rst), .send(send), .enable(enable), .throttle(throttle),
        .esc_signal(signal_out), .busy(busy)
    );

    task check_packet;
        input [10:0] value;
        input armed;
        input [15:0] expected;
        input perturb;
        begin
            @(negedge clk);
            throttle = value; enable = armed; send = 1;
            @(posedge clk); #1;
            decoded = 0;
            for (bit_number = 15; bit_number >= 0; bit_number = bit_number - 1) begin
                high_count = 0;
                for (cycle = 0; cycle < BITS; cycle = cycle + 1) begin
                    if (signal_out === 1'b1) begin
                        if (high_count != cycle) $fatal(1, "Noncontiguous high pulse");
                        high_count = high_count + 1;
                    end else if (signal_out !== 1'b0) $fatal(1, "Unknown ESC signal");
                    if (!busy) $fatal(1, "busy dropped during packet");
                    // Change commands and try send again while a packet is active.
                    @(negedge clk);
                    send = perturb;
                    if (perturb) begin throttle = 11'd2047; enable = 0; end
                    @(posedge clk); #1;
                end
                if (high_count == H1) decoded[bit_number] = 1;
                else if (high_count != H0) $fatal(1, "Wrong DShot pulse width: %0d", high_count);
            end
            if (decoded !== expected)
                $fatal(1, "DShot%0d decoded %h, expected %h", RATE / 1000, decoded, expected);
            for (cycle = 0; cycle < 2 * BITS; cycle = cycle + 1) begin
                if (signal_out !== 0 || !busy) $fatal(1, "Missing mandatory low gap");
                @(negedge clk); send = 0;
                @(posedge clk); #1;
            end
            if (busy || signal_out) $fatal(1, "Driver did not return to idle");
        end
    endtask

    initial begin
        repeat (2) @(negedge clk);
        if (signal_out !== 0 || busy !== 0) $fatal(1, "Reset failed");
        rst = 0;
        check_packet(0, 1, 16'h0000, 0);
        check_packet(1, 1, 16'h0000, 0);
        check_packet(47, 1, 16'h0000, 0);
        check_packet(48, 1, 16'h0606, 0);
        check_packet(1000, 1, 16'h7d0a, 1);
        check_packet(2047, 1, 16'hffee, 0);
        check_packet(2047, 0, 16'h0000, 0);
        // Reset truncates a packet and returns the line low on the next clock.
        @(negedge clk); enable = 1; send = 1;
        repeat (5) @(negedge clk);
        rst = 1; send = 0;
        @(posedge clk); #1;
        if (busy || signal_out) $fatal(1, "Mid-packet reset failed");
        @(negedge clk); rst = 0;
        check_packet(48, 1, 16'h0606, 0);
        done = 1;
    end
endmodule

module MotorDriver_tb;
    wire done150, done300, done600;
    MotorDriverChecks #(.RATE(150_000)) d150 (.done(done150));
    MotorDriverChecks #(.RATE(300_000)) d300 (.done(done300));
    MotorDriverChecks #(.RATE(600_000)) d600 (.done(done600));
    initial begin
        $dumpfile("MotorDriver.vcd"); $dumpvars(0, MotorDriver_tb);
        wait (done150 && done300 && done600);
        $display("PASS: DShot150/300/600 timing, golden packets, stop, latching, busy and reset");
        $finish;
    end
    initial begin #5_000_000; $fatal(1, "MotorDriver test timeout"); end
endmodule
