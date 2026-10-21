`include "../../../Drone/PeripheralInterface/RadioReceiver/IBusDecoder.v"
`timescale 1ns / 1ps
module IBusDecoder_tb;
    reg clk = 0, rst = 1, data_valid = 0, framing_error = 0;
    reg [7:0] data = 0;
    wire [223:0] channels;
    wire frame_valid, rc_valid;
    integer frames = 0, i;
    reg [223:0] previous_channels = 0;
    reg previous_frame = 0;
    always #500 clk = ~clk;
    IBusDecoder #(.CLK_FREQ(1_000_000), .TIMEOUT_MS(1), .BYTE_TIMEOUT_US(50)) dut (
        .clk(clk), .rst(rst), .data(data), .data_valid(data_valid),
        .framing_error(framing_error), .channels(channels),
        .frame_valid(frame_valid), .rc_valid(rc_valid)
    );
    initial begin $dumpfile("IBusDecoder.vcd"); $dumpvars(0, IBusDecoder_tb); end
    always @(posedge clk) begin
        #1;
        if (!rst) begin
            if (frame_valid) begin
                if (previous_frame || channels !== {14{16'd1500}} || !rc_valid)
                    $fatal(1, "Decoder commit incorrect");
                frames = frames + 1;
            end else if (channels !== previous_channels)
                $fatal(1, "Partial packet changed live channels");
        end
        previous_channels = channels;
        previous_frame = frame_valid;
    end
    task send_byte;
        input [7:0] value;
        begin
            @(negedge clk); data = value; data_valid = 1;
            @(negedge clk); data_valid = 0;
        end
    endtask
    task send_frame;
        input corrupt;
        integer ch;
        begin
            send_byte(8'h20); send_byte(8'h40);
            for (ch = 0; ch < 14; ch = ch + 1) begin
                send_byte(8'hdc); send_byte(8'h05);
            end
            // Known independent vector for 14 x 1500.
            send_byte(corrupt ? 8'h50 : 8'h51); send_byte(8'hf3);
        end
    endtask
    initial begin
        #2000; rst = 0;
        send_byte(8'h99); send_byte(8'h20); send_byte(8'h41);
        send_frame(1);
        if (frames != 0 || rc_valid !== 0) $fatal(1, "Malformed frame accepted");
        send_byte(8'h20); // Overlapping header.
        send_frame(0);
        if (frames != 1) $fatal(1, "Good frame rejected");
        // Frame committed on the preceding rising edge; test the exact timeout.
        repeat (999) @(negedge clk);
        if (rc_valid !== 1) $fatal(1, "Timeout fired before 1000 clocks");
        @(negedge clk);
        if (rc_valid !== 0 || channels !== {14{16'd1500}})
            $fatal(1, "Timeout boundary or retained channels incorrect");
        send_byte(8'h20); send_byte(8'h40); send_byte(8'hdc);
        repeat (60) @(negedge clk);
        send_frame(0);
        if (frames != 2) $fatal(1, "Gap recovery failed");
        send_byte(8'h20); send_byte(8'h40);
        @(negedge clk); framing_error = 1;
        @(negedge clk); framing_error = 0;
        send_frame(0);
        if (frames != 3) $fatal(1, "Framing error recovery failed");
        send_frame(1);
        if (frames != 3) $fatal(1, "Bad checksum accepted after valid frame");
        @(negedge clk); rst = 1;
        @(negedge clk);
        if (rc_valid !== 0 || frame_valid !== 0 || channels !== 0)
            $fatal(1, "Decoder reset failed");
        $display("PASS: IBusDecoder headers, checksum, atomic commit, exact timeout, recovery, reset");
        $finish;
    end
    initial begin #5000000; $fatal(1, "Decoder watchdog"); end
endmodule
