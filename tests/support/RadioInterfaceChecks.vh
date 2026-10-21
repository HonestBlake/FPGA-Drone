// Shared serial stimulus and assertions for the two enclosing module interfaces.
// Included inside a testbench module; contains no design module.
reg clk = 0, rst = 1, ibus_rx = 1;
wire [9:0] throttle;
wire signed [10:0] roll_command, pitch_command, yaw_command;
wire arm_request, avoidance_enable, frame_valid, rc_valid;
wire [223:0] channels;
reg [7:0] packet [0:31];
reg [223:0] expected_channels = 0, previous_channels = 0;
reg previous_frame = 0;
integer frame_count = 0, i, sum;
always #50 clk = ~clk;
always @(posedge clk) begin
    #1;
    if (!rst) begin
        if (frame_valid) begin
            if (previous_frame || channels !== expected_channels)
                $fatal(1, "Parent interface frame or atomic channels incorrect");
            frame_count = frame_count + 1;
        end else if (channels !== previous_channels)
            $fatal(1, "Parent channels updated without a valid frame");
    end
    previous_frame = frame_valid;
    previous_channels = channels;
end
task send_byte;
    input [7:0] value;
    integer bit_number;
    begin
        ibus_rx = 0; #1000;
        for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1) begin
            ibus_rx = value[bit_number]; #1000;
        end
        ibus_rx = 1; #1000;
    end
endtask
task set_channel;
    input integer ch;
    input [15:0] value;
    begin
        packet[ch*2] = value[7:0];
        packet[ch*2+1] = value[15:8];
    end
endtask
task send_frame;
    integer byte_number;
    begin
        for (byte_number = 0; byte_number < 32; byte_number = byte_number + 1)
            send_byte(packet[byte_number]);
        #1000;
    end
endtask
initial begin
    #203;
    @(negedge clk); rst = 0; #17;
    if (rc_valid !== 0) $fatal(1, "Parent RC valid before reception");
    packet[0] = 8'h20; packet[1] = 8'h40;
    for (i = 1; i <= 14; i = i + 1) set_channel(i, 1500);
    set_channel(14, 2000); // Non-default roll mapping.
    set_channel(2, 1250);  // Non-default throttle.
    set_channel(3, 1000);  // Non-default pitch.
    set_channel(4, 1750);
    set_channel(6, 2000);  // Non-default arm.
    set_channel(5, 1000);  // Non-default mode.
    sum = 0;
    for (i = 0; i < 30; i = i + 1) sum = sum + packet[i];
    sum = 65535 - sum;
    packet[30] = sum & 255; packet[31] = (sum >> 8) & 255;
    for (i = 0; i < 28; i = i + 1) expected_channels[i*8 +: 8] = packet[i+2];
    send_frame;
    if (frame_count != 1 || rc_valid !== 1 || throttle !== 250 ||
        roll_command !== 11'sd500 || pitch_command !== -11'sd500 ||
        yaw_command !== 11'sd250 || arm_request !== 1 || avoidance_enable !== 0)
        $fatal(1, "Parent serial parameter forwarding or control routing failed");
    packet[31] = packet[31] ^ 8'h01;
    send_frame;
    if (frame_count != 1) $fatal(1, "Parent accepted corrupt checksum");
    #1010000;
    if (rc_valid !== 0 || {throttle, roll_command, pitch_command, yaw_command,
                          arm_request, avoidance_enable} !== 45'b0)
        $fatal(1, "Parent timeout parameter forwarding or invalid gating failed");
    packet[31] = packet[31] ^ 8'h01;
    send_frame;
    if (frame_count != 2 || rc_valid !== 1) $fatal(1, "Parent recovery failed");
`ifdef DRONE_FLIGHT_TEST
    run_flight_checks;
`endif
    @(negedge clk); rst = 1;
    @(negedge clk);
    if (rc_valid !== 0 || frame_valid !== 0 || channels !== 0)
        $fatal(1, "Parent reset propagation failed");
    $display("PASS: %m serial integration, parameter forwarding, checksum, timeout, reset");
    $finish;
end
initial begin #40_000_000; $fatal(1, "Parent test watchdog"); end
