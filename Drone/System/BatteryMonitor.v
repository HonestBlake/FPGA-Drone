`timescale 1ns / 1ps

// 7-series XADC, single-ended 0..1 V input representing a 0..16 V pack.
// Connect battery_vin_n to analog ground. Never connect pack voltage directly.
module BatteryMonitor #(
    parameter integer CLK_FREQ = 100_000_000,
    parameter integer XADC_CHANNEL = 6,
    parameter integer EMPTY_MV = 9000, // 3S LiPo: 3 x 3.0 V discharge floor
    parameter integer FULL_MV = 12600,
    parameter integer FILTER_SAMPLES = 64, // Power of two; average complete ADC readings.
    parameter integer UPDATE_HZ = 4 // Stable human-readable output cadence.
) (
    input wire clk,
    input wire rst,
    input wire battery_vin_p,
    input wire battery_vin_n,
    output reg [11:0] battery_raw,
    output reg [15:0] battery_mv,
    output reg [6:0] battery_percent,
    output reg [9:0] battery_percent_tenths,
    output reg battery_valid,
    output reg battery_sample_valid
);
    localparam integer ADC_DIV = (CLK_FREQ + 25_999_999) / 26_000_000;
    localparam [15:0] CONFIG_0 = 16'h3110 + XADC_CHANNEL; // 256-sample XADC average, ACQ, unipolar, continuous
    localparam [15:0] CONFIG_2 = ((ADC_DIV < 2 ? 2 : ADC_DIV) << 8);
    localparam integer SPAN_MV = FULL_MV > EMPTY_MV ? FULL_MV - EMPTY_MV : 1;
    localparam [6:0] ADC_ADDRESS = 7'h10 + XADC_CHANNEL;
    wire [15:0] aux_p = {15'b0, battery_vin_p} << XADC_CHANNEL;
    wire [15:0] aux_n = {15'b0, battery_vin_n} << XADC_CHANNEL;
    wire [15:0] adc_data;
    wire eoc, drdy;
    // XADC codes are left aligned; one count is 1/4096 V at the pin,
    // or 16000/4096 mV at the pack. Round to the nearest millivolt.
    localparam integer UPDATE_CYCLES = CLK_FREQ/UPDATE_HZ;
    reg [31:0] filter_sum, filter_count, update_count;
    reg [11:0] filtered_raw;
    reg filtered_valid;
    wire [31:0] next_sum = filter_sum + {20'b0,adc_data[15:4]};
    wire [31:0] averaged_raw = (next_sum + FILTER_SAMPLES/2)/FILTER_SAMPLES;
    wire [31:0] scaled_mv = (filtered_raw * 32'd16000 + 32'd2048) >> 12;

    XADC #(
        .INIT_40(CONFIG_0), .INIT_41(16'h31af), .INIT_42(CONFIG_2)
    ) xadc (
        .DCLK(clk), .RESET(rst), .DADDR(ADC_ADDRESS),
        .DEN(eoc && !rst), .DWE(1'b0), .DI(16'b0), .DO(adc_data), .DRDY(drdy),
        .EOC(eoc), .EOS(), .BUSY(), .CHANNEL(), .ALM(), .OT(),
        .JTAGBUSY(), .JTAGLOCKED(), .JTAGMODIFIED(), .MUXADDR(),
        .CONVST(1'b0), .CONVSTCLK(1'b0), .VP(1'b0), .VN(1'b0),
        .VAUXP(aux_p), .VAUXN(aux_n)
    );

    // synthesis translate_off
    initial begin
        if (XADC_CHANNEL < 0 || XADC_CHANNEL > 15)
            $fatal(1, "BatteryMonitor: XADC_CHANNEL must be 0..15");
        if (CLK_FREQ <= 0 || ADC_DIV > 255)
            $fatal(1, "BatteryMonitor: unsupported CLK_FREQ");
        if (EMPTY_MV < 0 || FULL_MV <= EMPTY_MV || FULL_MV > 16000)
            $fatal(1, "BatteryMonitor: require 0 <= EMPTY_MV < FULL_MV <= 16000");
        if(FILTER_SAMPLES<1 || FILTER_SAMPLES>65536 ||
           (FILTER_SAMPLES & (FILTER_SAMPLES-1))!=0 || UPDATE_HZ<1 || UPDATE_HZ>CLK_FREQ)
            $fatal(1,"BatteryMonitor: power-of-two filter and valid update rate required");
    end
    // synthesis translate_on

    always @(posedge clk) begin
        if (rst) begin
            battery_raw <= 0;
            battery_mv <= 0;
            battery_percent <= 0;
            battery_percent_tenths <= 0;
            battery_valid <= 0;
            battery_sample_valid <= 0;
            filter_sum <= 0; filter_count <= 0; update_count <= 0;
            filtered_raw <= 0; filtered_valid <= 0;
        end else begin
            battery_sample_valid <= 0;
            if (drdy) begin
                if(filter_count == FILTER_SAMPLES-1) begin
                    filtered_raw <= averaged_raw[11:0]; filtered_valid <= 1;
                    filter_count <= 0; filter_sum <= 0;
                end else begin filter_sum <= next_sum; filter_count <= filter_count+1'b1; end
            end
            if(update_count == UPDATE_CYCLES-1) begin
                update_count <= 0;
                if(filtered_valid) begin
                    battery_raw <= filtered_raw;
                    battery_mv <= scaled_mv[15:0];
                    if (scaled_mv <= EMPTY_MV) battery_percent <= 0;
                    else if (scaled_mv >= FULL_MV) battery_percent <= 100;
                    else battery_percent <= ((scaled_mv - EMPTY_MV) * 100 + SPAN_MV/2) / SPAN_MV;
                    if (scaled_mv <= EMPTY_MV) battery_percent_tenths <= 0;
                    else if (scaled_mv >= FULL_MV) battery_percent_tenths <= 1000;
                    else battery_percent_tenths <= ((scaled_mv - EMPTY_MV) * 1000 + SPAN_MV/2) / SPAN_MV;
                    battery_valid <= 1;
                    battery_sample_valid <= 1;
                end
            end else update_count <= update_count+1'b1;
        end
    end
endmodule
