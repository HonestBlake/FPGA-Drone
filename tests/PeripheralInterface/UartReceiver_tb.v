`include "../../Drone/PeripheralInterface/UartReceiver.v"
`timescale 1ns / 1ps
module UartReceiver_tb;
    reg clk = 0, rst = 1, rx = 1;
    wire [7:0] data;
    wire data_valid, framing_error;
    reg [7:0] expected;
    reg last_valid = 0;
    integer received = 0, errors = 0, i;
    always #50 clk = ~clk;
    UartReceiver #(.CLK_FREQ(10_000_000), .BAUD_RATE(1_000_000)) dut (
        .clk(clk), .rst(rst), .rx(rx), .data(data),
        .data_valid(data_valid), .framing_error(framing_error)
    );
    initial begin
        $dumpfile("UartReceiver.vcd");
        $dumpvars(0, UartReceiver_tb);
    end
    always @(posedge clk) begin
        #1;
        if (!rst) begin
            if (data_valid) begin
                if (last_valid || data !== expected)
                    $fatal(1, "UART byte or data_valid pulse incorrect");
                received = received + 1;
            end
            if (framing_error) errors = errors + 1;
        end
        last_valid = data_valid;
    end
    task send_byte;
        input [7:0] value;
        input bad_stop;
        integer bit_number;
        begin
            expected = value;
            rx = 0; #1000;
            for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1) begin
                rx = value[bit_number]; #1000;
            end
            rx = !bad_stop; #1000;
            rx = 1;
        end
    endtask
    initial begin
        #203; rst = 0; #1000;
        rx = 0; #200; rx = 1; #2000;
        if (received != 0 || errors != 0) $fatal(1, "Short glitch accepted");
        for (i = 0; i < 256; i = i + 1) send_byte(i, 0);
        if (received != 256) $fatal(1, "Lost back-to-back UART byte");
        send_byte(8'h55, 1);
        rx = 0; #20000;
        if (received != 256 || errors != 1) $fatal(1, "Bad stop or break handling");
        rx = 1; #2000;
        send_byte(8'ha5, 0);
        if (received != 257) $fatal(1, "UART did not recover from break");
        rx = 0; #1000;
        rst = 1; #300;
        if (data_valid !== 0 || framing_error !== 0 || data !== 0)
            $fatal(1, "UART reset failed");
        rx = 1; rst = 0; #2000;
        send_byte(8'h5a, 0);
        if (received != 258) $fatal(1, "UART did not recover from reset");
        $display("PASS: UartReceiver all 256 bytes, glitches, bad stop, break, reset");
        $finish;
    end
    initial begin #5000000; $fatal(1, "UART watchdog"); end
endmodule
