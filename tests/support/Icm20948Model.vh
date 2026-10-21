// Simulation-only banked register slave. Checks mode-0 timing at the pins.
`timescale 1ns / 1ps
module Icm20948Model #(
    parameter integer MISO_DELAY_NS = 59
)(input wire reset, cs_n, sclk, mosi, output reg miso);
    reg [7:0] registers [0:511];
    reg [1:0] bank = 2;
    reg [7:0] who = 8'hea;
    reg ready_enable = 1, ignore_config = 0, disconnected = 0;
    reg signed [15:0] ax = 0, ay = 0, az = 16384, gx = 0, gy = 0, gz = 0;
    reg [7:0] shift, tx, command;
    reg [6:0] address;
    integer bit_index = 0, bytes = 0, reads = 0, resets = 0, i;
    time deselected_at = 0, selected_at = 0, last_rise = 0, last_fall = 0;
    function [7:0] read_register;
        input [6:0] addr;
        begin
            if (disconnected) read_register = 8'hff;
            else if (addr == 7'h7f) read_register = {2'b00,bank,4'b0000};
            else if (bank == 0) begin
                case (addr)
                    0: read_register = who;
                    7'h1a: read_register = ready_enable ? 8'h01 : 8'h00;
                    7'h2d: read_register = ax[15:8]; 7'h2e: read_register = ax[7:0];
                    7'h2f: read_register = ay[15:8]; 7'h30: read_register = ay[7:0];
                    7'h31: read_register = az[15:8]; 7'h32: read_register = az[7:0];
                    7'h33: read_register = gx[15:8]; 7'h34: read_register = gx[7:0];
                    7'h35: read_register = gy[15:8]; 7'h36: read_register = gy[7:0];
                    7'h37: read_register = gz[15:8]; 7'h38: read_register = gz[7:0];
                    default: read_register = registers[addr];
                endcase
            end else read_register = registers[bank*128+addr];
        end
    endfunction
    initial begin
        miso = 0;
        for (i=0;i<512;i=i+1) registers[i] = 0;
        registers[6] = 8'h41;
    end
    always @(negedge cs_n) begin
        if ($time-deselected_at < 50) $fatal(1, "ICM model: CS high time <50 ns");
        if (sclk !== 0) $fatal(1, "ICM model: SCLK not idle low");
        selected_at = $time; bit_index = 0; bytes = 0; shift = 0; tx = 0; miso = 0;
    end
    always @(posedge sclk) if (!cs_n) begin
        if ($time-selected_at < 8 || (last_fall != 0 && $time-last_fall < 64))
            $fatal(1, "ICM model: SPI setup/low time violation");
        last_rise = $time;
        shift = {shift[6:0],mosi};
        if (bit_index == 7) begin
            bit_index = 0;
            if (bytes == 0) begin
                command = shift; address = shift[6:0];
                tx = read_register(address);
            end else begin
                if (!command[7] && !disconnected) begin
                    if (address == 7'h7f) bank = shift[5:4];
                    else if (bank == 0 && address == 6 && shift[7]) begin
                        bank = 0; resets = resets+1;
                        for (i=0;i<512;i=i+1) registers[i] = 0;
                        registers[6] = 8'h41;
                    end else if (!(ignore_config && bank == 2 && (address == 1 || address == 7'h14)))
                        registers[bank*128+address] = shift;
                end
                address = address+1'b1; tx = read_register(address);
            end
            bytes = bytes+1;
        end else bit_index = bit_index+1;
    end
    always @(negedge sclk) if (!cs_n) begin
        if ($time-last_rise < 64) $fatal(1, "ICM model: SPI high time violation");
        last_fall = $time; miso <= #MISO_DELAY_NS tx[7-bit_index];
    end
    always @(posedge cs_n) begin
        deselected_at = $time;
        if (!reset && selected_at != 0) begin
            if (sclk !== 0 || $time-last_fall < 500 || bit_index != 0 || bytes < 2)
                $fatal(1, "ICM model: malformed SPI transaction");
            if (command == 8'had) begin
                if (bank != 0 || bytes != 13) $fatal(1, "ICM model: invalid sensor burst");
                reads = reads+1;
            end
        end
        miso = 0;
    end
endmodule
