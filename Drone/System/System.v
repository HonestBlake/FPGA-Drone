`timescale 1ns / 1ps

// System timing, battery monitoring, beacon, host UART, and Basys3 display.
module System #(
    parameter integer CLK_FREQ = 100_000_000,
    parameter integer CONTROL_TICK_FREQ = 1_000,
    parameter integer UART_BAUD_RATE = 115_200,
    parameter integer UART_FIFO_DEPTH = 16,
    parameter integer BATTERY_XADC_CHANNEL = 6,
    parameter integer BATTERY_EMPTY_MV = 9000,
    parameter integer BATTERY_FULL_MV = 12600,
    parameter integer BATTERY_FILTER_SAMPLES = 64,
    parameter integer BATTERY_UPDATE_HZ = 4,
    parameter integer LED_FLASHES_PER_MINUTE = 60,
    parameter integer LED_PULSE_MS = 100,
    parameter integer DISPLAY_REFRESH_HZ = 250,
    parameter integer DISPLAY_BLANK_CYCLES = (CLK_FREQ / 100_000 < 2) ? 2 : CLK_FREQ / 100_000
) (
    input wire clk,
    input wire rst,
    output wire control_tick,
    input wire [7:0] uart_data,
    input wire uart_valid,
    output wire uart_ready,
    output wire uart_busy,
    output wire uart_tx,
    input wire battery_vin_p,
    input wire battery_vin_n,
    output wire [11:0] battery_raw,
    output wire [15:0] battery_mv,
    output wire [6:0] battery_percent,
    output wire battery_valid,
    output wire battery_sample_valid,
    output wire beacon_led,
    input wire display_write,
    input wire [15:0] display_value,
    input wire [3:0] display_digit_enable,
    input wire [3:0] display_decimal_points,
    output wire [6:0] seg,
    output wire dp,
    output wire [3:0] an
);
    BatteryMonitor #(
        .CLK_FREQ(CLK_FREQ), .XADC_CHANNEL(BATTERY_XADC_CHANNEL),
        .EMPTY_MV(BATTERY_EMPTY_MV), .FULL_MV(BATTERY_FULL_MV),
        .FILTER_SAMPLES(BATTERY_FILTER_SAMPLES), .UPDATE_HZ(BATTERY_UPDATE_HZ)
    ) battery_monitor (
        .clk(clk), .rst(rst), .battery_vin_p(battery_vin_p), .battery_vin_n(battery_vin_n),
        .battery_raw(battery_raw), .battery_mv(battery_mv), .battery_percent(battery_percent),
        .battery_valid(battery_valid), .battery_sample_valid(battery_sample_valid)
    );

    LedBlinker #(
        .CLK_FREQ(CLK_FREQ), .FLASHES_PER_MINUTE(LED_FLASHES_PER_MINUTE),
        .PULSE_MS(LED_PULSE_MS)
    ) led_blinker (.clk(clk), .rst(rst), .led(beacon_led));

    TickGenerator #(.CLK_FREQ(CLK_FREQ), .TICK_FREQ(CONTROL_TICK_FREQ)) tick_generator (
        .clk(clk), .rst(rst), .tick(control_tick)
    );

    UartOutput #(
        .CLK_FREQ(CLK_FREQ), .BAUD_RATE(UART_BAUD_RATE), .FIFO_DEPTH(UART_FIFO_DEPTH)
    ) uart_output (
        .clk(clk), .rst(rst), .data(uart_data), .valid(uart_valid),
        .ready(uart_ready), .busy(uart_busy), .tx(uart_tx)
    );

    SevenSegmentDisplay #(
        .CLK_FREQ(CLK_FREQ), .REFRESH_HZ(DISPLAY_REFRESH_HZ),
        .BLANK_CYCLES(DISPLAY_BLANK_CYCLES)
    ) seven_segment_display (
        .clk(clk), .rst(rst), .write(display_write), .value(display_value),
        .digit_enable(display_digit_enable), .decimal_points(display_decimal_points),
        .seg(seg), .dp(dp), .an(an)
    );
endmodule
