`timescale 1ns / 1ps

// Standalone Basys3 radio test. Receiver TX enters the FPGA on tx (JC1).
// Exactly one switch high: SW0 throttle, SW1 roll, SW2 pitch, SW3 yaw.
module RadioDisplayTest #(
    parameter integer CLK_FREQ = 100_000_000,
    parameter integer BAUD_RATE = 115_200,
    parameter integer TIMEOUT_MS = 100,
    parameter integer DISPLAY_REFRESH_HZ = 250
) (
    input wire clk,
    input wire btnC,
    input wire tx,
    input wire [3:0] sw,
    output wire [6:0] seg,
    output wire dp,
    output wire [3:0] an
);
    // Vivado implements these initial values at FPGA configuration. Hold all
    // synchronous-reset modules in reset while the input synchronizer settles.
    reg [7:0] startup_reset = 8'hff;
    (* ASYNC_REG = "TRUE" *) reg reset_meta = 1'b1, reset_sync = 1'b1;
    (* ASYNC_REG = "TRUE" *) reg [3:0] sw_meta, sw_sync;
    wire rst = (|startup_reset) || reset_sync;

    always @(posedge clk) begin
        startup_reset <= {startup_reset[6:0], 1'b0};
        reset_meta <= btnC;
        reset_sync <= reset_meta;
        if (rst) begin
            sw_meta <= 0;
            sw_sync <= 0;
        end else begin
            sw_meta <= sw;
            sw_sync <= sw_meta;
        end
    end

    wire [9:0] throttle;
    wire signed [10:0] roll_command, pitch_command, yaw_command;
    wire rc_valid;
    RadioReceiver #(
        .CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE), .TIMEOUT_MS(TIMEOUT_MS)
    ) radio (
        .clk(clk), .rst(rst), .ibus_rx(tx), .throttle(throttle),
        .roll_command(roll_command), .pitch_command(pitch_command),
        .yaw_command(yaw_command), .arm_request(), .avoidance_enable(),
        .frame_valid(), .rc_valid(rc_valid), .channels()
    );

    reg signed [10:0] selected_command;
    reg selection_valid;
    reg [10:0] magnitude;
    reg [26:0] bcd_shift;
    reg [15:0] display_value;
    integer bit_index, digit_index;

    always @* begin
        selection_valid = 1'b1;
        case (sw_sync)
            4'b0001: selected_command = $signed({1'b0, throttle});
            4'b0010: selected_command = roll_command;
            4'b0100: selected_command = pitch_command;
            4'b1000: selected_command = yaw_command;
            default: begin
                selected_command = 0;
                selection_valid = 1'b0;
            end
        endcase

        magnitude = selected_command[10] ? -selected_command : selected_command;
        // Eleven-bit binary to four decimal digits (shift/add-three).
        bcd_shift = {16'b0, magnitude};
        for (bit_index = 0; bit_index < 11; bit_index = bit_index + 1) begin
            for (digit_index = 0; digit_index < 4; digit_index = digit_index + 1)
                if (bcd_shift[11 + digit_index*4 +: 4] >= 5)
                    bcd_shift[11 + digit_index*4 +: 4] =
                        bcd_shift[11 + digit_index*4 +: 4] + 4'd3;
            bcd_shift = bcd_shift << 1;
        end
        display_value = bcd_shift[26:11];
        // Axes fit three magnitude digits; reserve the left digit for '-'.
        if (selected_command[10]) display_value[15:12] = 4'hf;
        // Idle indicator works without a receiver or a recent valid packet.
        if (sw_sync == 4'b0000) display_value = 16'hffff;
    end

    SevenSegmentDisplay #(
        .CLK_FREQ(CLK_FREQ), .REFRESH_HZ(DISPLAY_REFRESH_HZ), .F_IS_MINUS(1)
    ) display (
        .clk(clk), .rst(rst), .write(1'b1), .value(display_value),
        .digit_enable(((sw_sync == 4'b0000) || (rc_valid && selection_valid))
                      ? 4'b1111 : 4'b0000),
        .decimal_points(4'b0000), .seg(seg), .dp(dp), .an(an)
    );
endmodule
