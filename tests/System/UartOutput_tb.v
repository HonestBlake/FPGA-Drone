`include "../../Drone/UartTransmitter.v"
`include "../../Drone/System/UartOutput.v"
`timescale 1ns / 1ps
module UartOutput_tb;
    reg clk = 0, rst = 1;
    reg [7:0] data = 0;
    reg valid = 0;
    wire ready, busy, tx;
    reg [7:0] expected [0:1023];
    reg [7:0] decoded;
    integer accepted = 0, received = 0, stalled = 0, simultaneous = 0, b, n;
    // Depth 3 specifically exercises wrap without relying on a power of two.
    UartOutput #(.CLK_FREQ(10_000_000), .BAUD_RATE(1_000_000), .FIFO_DEPTH(3)) dut (
        .clk(clk), .rst(rst), .data(data), .valid(valid),
        .ready(ready), .busy(busy), .tx(tx)
    );
    always #50 clk = ~clk;
    always @(posedge clk) begin
        if (!rst) begin
            if (valid && ready) begin
                expected[accepted] = data;
                accepted = accepted + 1;
            end
            if (valid && !ready) stalled = stalled + 1;
            if (dut.push && dut.pop) simultaneous = simultaneous + 1;
        end
    end
    initial forever begin
        @(negedge tx);
        if (!rst) begin : decode_byte
            // Abort decoding on reset, just as a receiver loses a partial byte.
            fork : decode_or_reset
                begin
                    #500;
                    if (tx !== 0) $fatal(1, "Queued UART start bit");
                    for (b = 0; b < 8; b = b + 1) begin
                        #1000; decoded[b] = tx;
                    end
                    #1000;
                    if (tx !== 1 || received >= accepted || decoded !== expected[received])
                        $fatal(1, "Queued UART dropped/reordered/corrupted byte %0d", received);
                    received = received + 1;
                    disable decode_or_reset;
                end
                begin wait (rst); disable decode_or_reset; end
            join
        end
    end
    task enqueue;
        input [7:0] value;
        begin
            @(negedge clk); valid = 1; data = value;
            @(posedge clk);
            while (!ready) @(posedge clk);
            @(negedge clk); valid = 0;
        end
    endtask
    initial begin
        $dumpfile("UartOutput.vcd"); $dumpvars(0, UartOutput_tb);
        repeat (3) @(negedge clk);
        if (ready !== 0 || busy !== 0 || tx !== 1) $fatal(1, "Queue reset");
        rst = 0;
        for (n = 0; n < 256; n = n + 1) enqueue(n);
        wait (!busy); #100;
        if (received != 256 || accepted != 256 || stalled == 0 || simultaneous == 0)
            $fatal(1, "Queue wrap/full/backpressure coverage");
        // Real text message including CR/LF.
        enqueue("O"); enqueue("K"); enqueue(13); enqueue(10);
        wait (!busy); #100;
        if (received != 260) $fatal(1, "ASCII message");
        enqueue(8'hff); enqueue(8'h00); enqueue(8'ha5);
        repeat (5) @(negedge clk);
        rst = 1;
        repeat (3) @(negedge clk);
        if (ready !== 0 || busy !== 0 || tx !== 1) $fatal(1, "Queue reset while busy");
        accepted = 0; received = 0;
        rst = 0;
        enqueue(8'h5a); wait (!busy); #100;
        if (received != 1) $fatal(1, "Queue reset retained old bytes");
        $display("PASS: UartOutput all bytes, ASCII, backpressure, simultaneous push/pop, wrap, reset");
        $finish;
    end
    initial begin #5000000; $fatal(1, "UartOutput watchdog"); end
endmodule
