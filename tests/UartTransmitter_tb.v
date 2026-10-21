`include "../Drone/UartTransmitter.v"
`timescale 1ns / 1ps
module UartTransmitter_tb;
    reg clk = 0, rst = 1, start = 0;
    reg [7:0] data = 0;
    wire tx, busy;
    integer value, bit_number, cycle;
    reg [9:0] expected;
    always #50 clk = ~clk;
    UartTransmitter #(.CLK_FREQ(10_000_000), .BAUD_RATE(1_000_000)) dut (
        .clk(clk), .rst(rst), .data(data), .start(start), .tx(tx), .busy(busy)
    );
    initial begin
        $dumpfile("UartTransmitter.vcd");
        $dumpvars(0, UartTransmitter_tb);
        repeat (3) @(negedge clk);
        if (tx !== 1 || busy !== 0) $fatal(1, "TX reset/idle");
        rst = 0;
        for (value = 0; value < 256; value = value + 1) begin
            data = value;
            expected = {1'b1, data, 1'b0};
            start = 1;
            @(negedge clk); start = 0;
            data = ~data; // The accepted byte must be latched.
            for (bit_number = 0; bit_number < 10; bit_number = bit_number + 1) begin
                for (cycle = 0; cycle < 10; cycle = cycle + 1) begin
                    if (tx !== expected[bit_number] || busy !== 1)
                        $fatal(1, "TX bit value/duration: byte %0d bit %0d", value, bit_number);
                    start = (bit_number == 4 && cycle == 0); // Ignore while busy.
                    @(negedge clk);
                end
            end
            start = 0;
            if (busy !== 0 || tx !== 1) $fatal(1, "TX stop/ready timing");
        end
        start = 1;
        @(negedge clk); start = 0;
        repeat (15) @(negedge clk);
        rst = 1;
        @(negedge clk);
        if (busy !== 0 || tx !== 1) $fatal(1, "TX mid-byte reset");
        rst = 0;
        repeat (110) @(negedge clk);
        if (busy !== 0 || tx !== 1) $fatal(1, "TX restarted aborted byte");
        $display("PASS: UartTransmitter all bytes, bit timing, busy rejection, reset");
        $finish;
    end
    initial begin #5000000; $fatal(1, "TX watchdog"); end
endmodule
