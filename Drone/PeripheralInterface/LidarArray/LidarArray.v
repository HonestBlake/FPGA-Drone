`timescale 1ns / 1ps

// SEN0628 binary request/response UART interface. rst is synchronous, active high.
// Protocol source and buffer contract are documented in README.md beside this file.
module LidarArray #(
    parameter integer CLK_FREQ = 100_000_000,
    parameter integer STARTUP_MS = 1000,
    parameter integer SETTLE_MS = 5000,
    parameter integer POLL_MS = 100,
    parameter integer RESPONSE_TIMEOUT_MS = 8000,
    parameter integer BYTE_TIMEOUT_MS = 500
) (
    input wire clk,
    input wire rst,
    input wire lidar_rx,
    output wire lidar_tx,
    input wire [5:0] read_address, // {row[2:0], column[2:0]} in sensor order.
    output wire [15:0] read_distance_mm, // Asynchronous read of published image.
    output reg image_valid, // At least one complete scan since reset; not freshness.
    output reg frame_valid, // One clock when the published bank changes.
    output reg [31:0] frame_count,
    output reg initialized,
    output reg error // One clock on protocol error, UART error, or timeout.
);
    // Divide before multiplying so the default five/eight-second delays do not
    // overflow a signed 32-bit intermediate at the 100 MHz system clock.
    function [31:0] cycles_ms;
        input integer ms;
        begin
            cycles_ms = (CLK_FREQ / 1000) * ms + ((CLK_FREQ % 1000) * ms) / 1000;
        end
    endfunction
    localparam [31:0] STARTUP_CYCLES = cycles_ms(STARTUP_MS);
    localparam [31:0] SETTLE_CYCLES = cycles_ms(SETTLE_MS);
    localparam [31:0] POLL_CYCLES = cycles_ms(POLL_MS);
    localparam [31:0] RESPONSE_CYCLES = cycles_ms(RESPONSE_TIMEOUT_MS);
    localparam [31:0] BYTE_CYCLES = cycles_ms(BYTE_TIMEOUT_MS);
    localparam [31:0] TX_GAP_CYCLES = cycles_ms(1);
    localparam [3:0] DELAY = 0, SEND = 1, TX_WAIT = 2, TX_GAP = 3,
        STATUS = 4, COMMAND = 5, LENGTH_LOW = 6, LENGTH_HIGH = 7,
        PIXELS = 8, ACK_DATA = 9, RECOVER = 10;
    reg [3:0] state;
    reg [31:0] delay_count, response_count, byte_count;
    reg [2:0] tx_index;
    reg tx_start;
    reg [7:0] tx_data;
    wire tx_busy;
    wire [7:0] rx_data;
    wire rx_valid, rx_error;
    reg [7:0] length_low, pixel_low;
    reg [15:0] ack_remaining;
    reg [6:0] pixel_byte;
    reg published_bank;
    // Two 64 x 16 banks: only a completed response changes the published bank.
    // RAM contents need no reset because image_valid masks them until publication.
    reg [15:0] image_buffer [0:127];
    assign read_distance_mm = image_valid ?
        image_buffer[{published_bank, read_address}] : 16'd0;
    wire receiving = state >= STATUS && state <= ACK_DATA;

    UartReceiver #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(115_200)) receiver (
        .clk(clk), .rst(rst), .rx(lidar_rx), .data(rx_data),
        .data_valid(rx_valid), .framing_error(rx_error)
    );
    UartTransmitter #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(115_200)) transmitter (
        .clk(clk), .rst(rst), .data(tx_data), .start(tx_start),
        .tx(lidar_tx), .busy(tx_busy)
    );

    // Request length includes the command byte, is big endian, and differs
    // from the response's little-endian payload length.
    always @* begin
        case (tx_index)
            0: tx_data = 8'h55;
            1: tx_data = 0;
            2: tx_data = initialized ? 8'h01 : 8'h05;
            3: tx_data = initialized ? 8'h02 : 8'h01;
            7: tx_data = 8'h08;
            default: tx_data = 0;
        endcase
    end

    // synthesis translate_off
    initial begin
        if (CLK_FREQ < 1_000_000 || STARTUP_MS < 1 || SETTLE_MS < 1 ||
            POLL_MS < 1 || RESPONSE_TIMEOUT_MS < 1 || BYTE_TIMEOUT_MS < 1 ||
            (64'd1 * CLK_FREQ * RESPONSE_TIMEOUT_MS / 1000) > 32'hffffffff ||
            (64'd1 * CLK_FREQ * BYTE_TIMEOUT_MS / 1000) > 32'hffffffff ||
            (64'd1 * CLK_FREQ * STARTUP_MS / 1000) > 32'hffffffff ||
            (64'd1 * CLK_FREQ * SETTLE_MS / 1000) > 32'hffffffff ||
            (64'd1 * CLK_FREQ * POLL_MS / 1000) > 32'hffffffff)
            $fatal(1, "LidarArray: invalid clock or delay parameters");
    end
    // synthesis translate_on

    always @(posedge clk) begin
        if (rst) begin
            state <= DELAY;
            delay_count <= STARTUP_CYCLES - 1;
            response_count <= 0;
            byte_count <= 0;
            tx_index <= 0;
            tx_start <= 0;
            length_low <= 0;
            pixel_low <= 0;
            pixel_byte <= 0;
            ack_remaining <= 0;
            published_bank <= 0;
            initialized <= 0;
            image_valid <= 0;
            frame_valid <= 0;
            frame_count <= 0;
            error <= 0;
        end else begin
            tx_start <= 0;
            frame_valid <= 0;
            error <= 0;
            if (receiving) begin
                response_count <= response_count + 1'b1;
                if (rx_valid) byte_count <= 0;
                else byte_count <= byte_count + 1'b1;
            end
            // A total deadline also bounds a stream that never stops sending.
            // Before the first status byte, allow the full response deadline.
            if (receiving && (rx_error || response_count >= RESPONSE_CYCLES - 1 ||
                (state != STATUS && !rx_valid && byte_count >= BYTE_CYCLES - 1))) begin
                error <= 1;
                initialized <= 0;
                delay_count <= BYTE_CYCLES - 1;
                state <= RECOVER;
            end else begin
                case (state)
                    DELAY: if (delay_count != 0)
                        delay_count <= delay_count - 1'b1;
                    else begin
                        tx_index <= 0;
                        state <= SEND;
                    end
                    SEND: begin
                        tx_start <= 1;
                        state <= TX_WAIT;
                    end
                    TX_WAIT: if (!tx_start && !tx_busy) begin
                        if (tx_index == (initialized ? 3 : 7)) begin
                            response_count <= 0;
                            byte_count <= 0;
                            state <= STATUS;
                        end else begin
                            tx_index <= tx_index + 1'b1;
                            delay_count <= TX_GAP_CYCLES - 1;
                            state <= TX_GAP;
                        end
                    end
                    TX_GAP: if (delay_count != 0)
                        delay_count <= delay_count - 1'b1;
                    else state <= SEND;
                    STATUS: if (rx_valid) begin
                        if (rx_data == 8'h53) state <= COMMAND;
                        else begin
                            error <= 1;
                            initialized <= 0;
                            delay_count <= BYTE_CYCLES - 1;
                            state <= RECOVER;
                        end
                    end
                    COMMAND: if (rx_valid) begin
                        if (rx_data == (initialized ? 8'h02 : 8'h01))
                            state <= LENGTH_LOW;
                        else begin
                            error <= 1;
                            initialized <= 0;
                            delay_count <= BYTE_CYCLES - 1;
                            state <= RECOVER;
                        end
                    end
                    LENGTH_LOW: if (rx_valid) begin
                        length_low <= rx_data;
                        state <= LENGTH_HIGH;
                    end
                    LENGTH_HIGH: if (rx_valid) begin
                        if (initialized && {rx_data, length_low} == 16'd128) begin
                            pixel_byte <= 0;
                            state <= PIXELS;
                        end else if (!initialized && {rx_data, length_low} < 16'd1000) begin
                            // The library accepts any successful mode ACK payload.
                            // Drain it before requesting an image.
                            if ({rx_data, length_low} == 0) begin
                                initialized <= 1;
                                delay_count <= SETTLE_CYCLES - 1;
                                state <= DELAY;
                            end else begin
                                ack_remaining <= {rx_data, length_low};
                                state <= ACK_DATA;
                            end
                        end else begin
                            error <= 1;
                            initialized <= 0;
                            delay_count <= BYTE_CYCLES - 1;
                            state <= RECOVER;
                        end
                    end
                    ACK_DATA: if (rx_valid) begin
                        if (ack_remaining == 1) begin
                            initialized <= 1;
                            delay_count <= SETTLE_CYCLES - 1;
                            state <= DELAY;
                        end else ack_remaining <= ack_remaining - 1'b1;
                    end
                    PIXELS: if (rx_valid) begin
                        if (!pixel_byte[0]) pixel_low <= rx_data;
                        else image_buffer[{!published_bank, pixel_byte[6:1]}] <=
                            {rx_data, pixel_low};
                        if (pixel_byte == 127) begin
                            published_bank <= !published_bank;
                            image_valid <= 1;
                            frame_valid <= 1;
                            frame_count <= frame_count + 1'b1;
                            delay_count <= POLL_CYCLES - 1;
                            state <= DELAY;
                        end else pixel_byte <= pixel_byte + 1'b1;
                    end
                    // Discard the rest of an invalid/truncated response; require
                    // a quiet byte-timeout interval before configuring again.
                    RECOVER: if (rx_valid || rx_error)
                        delay_count <= BYTE_CYCLES - 1;
                    else if (delay_count != 0)
                        delay_count <= delay_count - 1'b1;
                    else begin
                        tx_index <= 0;
                        state <= SEND;
                    end
                    default: begin
                        initialized <= 0;
                        delay_count <= STARTUP_CYCLES - 1;
                        state <= DELAY;
                    end
                endcase
            end
        end
    end
endmodule
