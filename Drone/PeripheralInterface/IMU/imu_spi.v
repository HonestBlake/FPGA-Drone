`timescale 1ns / 1ps

module imu_spi #(
    parameter integer CLK_FREQ = 100_000_000,
    parameter integer CLK_DIV = (CLK_FREQ + 9_999_999) / 10_000_000,
    parameter integer RESET_CYCLES = (CLK_FREQ / 1000) * 100,

    // Basys 3 = 100 MHz.
    // Datasheet allows up to 100 ms before register access after power-up.
    parameter POWERUP_CYCLES = (CLK_FREQ / 1000) * 100,

    // Gyro startup from sleep can take about 35 ms.
    parameter WAKE_CYCLES = (CLK_FREQ / 1000) * 35
)(
    input  wire clk,
    input  wire reset,
    input  wire start,

    // SPI
    output reg  spi_cs_n,
    output reg  spi_sclk,
    output reg  spi_mosi,
    input  wire spi_miso,

    // Sensor outputs
    output reg signed [15:0] accel_x,
    output reg signed [15:0] accel_y,
    output reg signed [15:0] accel_z,

    output reg signed [15:0] gyro_x,
    output reg signed [15:0] gyro_y,
    output reg signed [15:0] gyro_z,

    // Status
    output reg data_valid,
    output reg busy,
    output reg init_done,
    output reg imu_detected,
    output reg error
);

    // =========================================================
    // ICM-20948 REGISTER ADDRESSES
    // =========================================================

    localparam WHO_EXPECTED   = 8'hEA;

    // =========================================================
    // SPI BYTE ENGINE
    // =========================================================

    reg [7:0] tx_byte;
    reg [7:0] rx_byte;
    reg [7:0] rx_shift;

    reg       byte_start;
    reg       byte_busy;
    reg       byte_done;

    reg [3:0] bit_count;
    reg [31:0] clk_count;

    // ---------------------------------------------------------
    // This block transmits/receives ONE SPI byte.
    //
    // SPI mode used:
    // SCLK idle low
    // MOSI changes while clock low
    // MISO sampled on rising edge
    // ---------------------------------------------------------

    always @(posedge clk) begin

        if (reset) begin

            spi_sclk  <= 1'b0;
            spi_mosi  <= 1'b0;

            byte_busy <= 1'b0;
            byte_done <= 1'b0;

            bit_count <= 0;
            clk_count <= 0;

            rx_shift <= 0;
            rx_byte  <= 0;

        end

        else begin

            // Normally low.
            // Goes high for one FPGA clock when a byte finishes.
            byte_done <= 1'b0;

            // -------------------------------------------------
            // Start a new byte
            // -------------------------------------------------

            if (byte_start && !byte_busy) begin

                byte_busy <= 1'b1;

                spi_sclk <= 1'b0;

                // SPI sends MSB first
                spi_mosi <= tx_byte[7];

                bit_count <= 0;
                clk_count <= 0;

                rx_shift <= 0;
            end

            // -------------------------------------------------
            // Byte currently transferring
            // -------------------------------------------------

            else if (byte_busy) begin

                if (clk_count == CLK_DIV - 1) begin

                    clk_count <= 0;

                    // =========================================
                    // Rising SPI edge
                    // =========================================

                    if (spi_sclk == 1'b0) begin

                        spi_sclk <= 1'b1;

                        // Sample MISO
                        rx_shift <= {
                            rx_shift[6:0],
                            spi_miso
                        };

                    end

                    // =========================================
                    // Falling SPI edge
                    // =========================================

                    else begin

                        spi_sclk <= 1'b0;

                        // Finished 8 bits?
                        if (bit_count == 7) begin

                            rx_byte <= rx_shift;

                            byte_busy <= 1'b0;
                            byte_done <= 1'b1;

                            spi_mosi <= 1'b0;

                        end

                        else begin

                            bit_count <= bit_count + 1'b1;

                            // Put next bit on MOSI
                            spi_mosi <=
                                tx_byte[6 - bit_count];

                        end
                    end

                end

                else begin
                    clk_count <= clk_count + 1'b1;
                end
            end
        end
    end


    // Transaction sequencer around the team member's mode-0 byte engine.
    // Delay CS transitions by a full half-period (>=50 ns at supported clocks).
    // Datasheet table 8 additionally requires 500 ns CS hold after transfer.
    localparam integer CS_HOLD_CYCLES = (CLK_FREQ + 1_999_999) / 2_000_000;
    localparam POWER_WAIT=0, RESET_WAIT=1, INIT=2, CMD=3, CMD_WAIT=4,
               DATA=5, DATA_WAIT=6, COMPLETE=7, GAP=8, WAKE_WAIT=9,
               READY=10, CHECK_ID=11, CHECK_READY=12, BURST=13, FAILED=14;
    reg [3:0] state, return_state;
    reg [31:0] wait_counter;
    reg [4:0] init_index;
    reg [7:0] command, write_value;
    reg [3:0] length, byte_count;
    reg [7:0] imu_data [0:11];
    reg [15:0] init_word;
    integer i;

    // Bank selection also works when an FPGA-only reset leaves the IMU in bank 2.
    // Entries with bit 15 set read back and compare against the low byte.
    always @* begin
        case (init_index)
            0: init_word = 16'h7f00;
            1: init_word = 16'h0680; // DEVICE_RESET; wait 100 ms afterwards
            2: init_word = 16'h7f00;
            3: init_word = 16'h0310; // disable primary I2C interface
            4: init_word = 16'h0601; // wake, auto PLL
            5: init_word = 16'h0700; // all accel/gyro axes enabled
            6: init_word = 16'h1101; // raw-data-ready status/interrupt enabled
            7: init_word = 16'h8310; // verify USER_CTRL
            8: init_word = 16'h8601; // verify wake
            9: init_word = 16'h7f20;
            10: init_word = 16'h0901; // align ODR start
            11: init_word = 16'h0001; // gyro 1125 / 2 Hz
            12: init_word = 16'h0119; // +/-250 dps, DLPF ~51 Hz
            13: init_word = 16'h1000;
            14: init_word = 16'h1101; // accel 1125 / 2 Hz
            15: init_word = 16'h1419; // +/-2g, DLPF ~50 Hz
            16: init_word = 16'h8119; // verify gyro configuration
            17: init_word = 16'h9419; // verify accel configuration
            18: init_word = 16'h7f00;
            default: init_word = 16'h80ea; // verify identity in bank 0
        endcase
    end

    always @(posedge clk) begin
        if (reset) begin
            state <= POWER_WAIT; return_state <= INIT;
            spi_cs_n <= 1; byte_start <= 0; tx_byte <= 0;
            wait_counter <= 0; init_index <= 0; command <= 0;
            write_value <= 0; length <= 1; byte_count <= 0;
            busy <= 1; data_valid <= 0; init_done <= 0;
            imu_detected <= 0; error <= 0;
            accel_x <= 0; accel_y <= 0; accel_z <= 0;
            gyro_x <= 0; gyro_y <= 0; gyro_z <= 0;
            for (i=0; i<12; i=i+1) imu_data[i] <= 0;
        end else begin
            byte_start <= 0; data_valid <= 0;
            case (state)
                POWER_WAIT, RESET_WAIT, WAKE_WAIT: begin
                    if ((state == POWER_WAIT && wait_counter >= POWERUP_CYCLES-1) ||
                        (state == RESET_WAIT && wait_counter >= RESET_CYCLES-1) ||
                        (state == WAKE_WAIT && wait_counter >= WAKE_CYCLES-1)) begin
                        wait_counter <= 0;
                        if (state == WAKE_WAIT) begin
                            init_done <= 1; busy <= 0; state <= READY;
                        end else state <= INIT;
                    end else wait_counter <= wait_counter + 1'b1;
                end
                INIT: begin
                    command <= init_word[15:8]; write_value <= init_word[7:0];
                    length <= 1; return_state <= INIT; state <= CMD;
                end
                CMD: begin
                    spi_cs_n <= 0; tx_byte <= command; byte_start <= 1;
                    byte_count <= 0; state <= CMD_WAIT;
                end
                CMD_WAIT: if (byte_done) state <= DATA;
                DATA: begin
                    tx_byte <= command[7] ? 8'h00 : write_value;
                    byte_start <= 1; state <= DATA_WAIT;
                end
                DATA_WAIT: if (byte_done) begin
                    imu_data[byte_count] <= rx_byte;
                    if (byte_count == length-1) begin
                        wait_counter <= 0; state <= COMPLETE;
                    end else begin byte_count <= byte_count+1'b1; state <= DATA; end
                end
                COMPLETE: begin
                    // Hold CS low beyond the final SCLK falling edge.
                    if (wait_counter >= CS_HOLD_CYCLES-1) begin
                        spi_cs_n <= 1; wait_counter <= 0; state <= GAP;
                    end else wait_counter <= wait_counter+1'b1;
                end
                GAP: begin
                    if (wait_counter >= CLK_DIV-1) begin
                        wait_counter <= 0;
                        case (return_state)
                            INIT: begin
                                if (command[7] && imu_data[0] !== write_value) state <= FAILED;
                                else if (init_index == 19) begin
                                    imu_detected <= 1; state <= WAKE_WAIT;
                                end else begin
                                    init_index <= init_index+1'b1;
                                    state <= (init_index == 1) ? RESET_WAIT : INIT;
                                end
                            end
                            CHECK_ID: begin
                                if (imu_data[0] !== WHO_EXPECTED) state <= FAILED;
                                else state <= CHECK_READY;
                            end
                            CHECK_READY: begin
                                if (imu_data[0][0]) state <= BURST;
                                else begin busy <= 0; state <= READY; end
                            end
                            BURST: begin
                                accel_x <= {imu_data[0],imu_data[1]};
                                accel_y <= {imu_data[2],imu_data[3]};
                                accel_z <= {imu_data[4],imu_data[5]};
                                gyro_x <= {imu_data[6],imu_data[7]};
                                gyro_y <= {imu_data[8],imu_data[9]};
                                gyro_z <= {imu_data[10],imu_data[11]};
                                data_valid <= 1; busy <= 0; state <= READY;
                            end
                            default: state <= FAILED;
                        endcase
                    end else wait_counter <= wait_counter+1'b1;
                end
                READY: if (start) begin
                    busy <= 1; command <= 8'h80; length <= 1;
                    return_state <= CHECK_ID; state <= CMD;
                end
                CHECK_READY: begin
                    command <= 8'h9a; length <= 1;
                    return_state <= CHECK_READY; state <= CMD;
                end
                BURST: begin
                    command <= 8'had; length <= 12;
                    return_state <= BURST; state <= CMD;
                end
                FAILED: begin
                    error <= 1; init_done <= 0; imu_detected <= 0;
                    busy <= 0; spi_cs_n <= 1;
                end
                default: state <= FAILED;
            endcase
        end
    end
    // synthesis translate_off
    initial begin
        if (CLK_FREQ < 1000 || CLK_DIV < 1 ||
            CLK_DIV < (CLK_FREQ + 13_999_999) / 14_000_000 ||
            POWERUP_CYCLES < 1 || RESET_CYCLES < 1 || WAKE_CYCLES < 1)
            $fatal(1, "imu_spi: invalid timing or SCLK exceeds 7 MHz");
    end
    // synthesis translate_on
endmodule
