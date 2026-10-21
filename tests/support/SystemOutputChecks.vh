// Included inside System_tb and Drone_tb. Both use 10 MHz / 1 Mbaud,
// display refresh 25 kHz (100 clocks per slot), and a 3-clock blank interval.
reg [7:0] uart_data = 0;
reg uart_valid = 0, display_write = 0;
reg [15:0] display_value = 0;
reg [3:0] display_digit_enable = 0, display_decimal_points = 0;
wire uart_ready, uart_busy, uart_tx, dp;
wire [6:0] seg;
wire [3:0] an;
reg system_checks_done = 0;
reg [7:0] serial_value;
integer sys_byte, sys_bit;
task system_send;
    input [7:0] value;
    begin
        @(negedge clk); uart_valid = 1; uart_data = value;
        @(posedge clk);
        while (!uart_ready) @(posedge clk);
        @(negedge clk); uart_valid = 0;
    end
endtask
initial begin
    wait (!rst);
    fork
        begin
            system_send("O"); system_send("K"); system_send(13); system_send(10);
            wait (!uart_busy);
        end
        begin
            for (sys_byte = 0; sys_byte < 4; sys_byte = sys_byte + 1) begin
                @(negedge uart_tx); #500;
                for (sys_bit = 0; sys_bit < 8; sys_bit = sys_bit + 1) begin
                    #1000; serial_value[sys_bit] = uart_tx;
                end
                #1000;
                if (uart_tx !== 1) $fatal(1, "System UART stop bit");
                case (sys_byte)
                    0: if (serial_value !== "O") $fatal(1, "System UART message O");
                    1: if (serial_value !== "K") $fatal(1, "System UART message K");
                    2: if (serial_value !== 13) $fatal(1, "System UART message CR");
                    3: if (serial_value !== 10) $fatal(1, "System UART message LF");
                endcase
            end
        end
        begin
            @(negedge clk);
            display_value = 16'h1234; display_digit_enable = 4'hf;
            display_decimal_points = 4'b0101; display_write = 1;
            @(negedge clk); display_write = 0;
            display_value = 0; display_decimal_points = 0; display_digit_enable = 0;
            wait (an == 4'he); #1;
            if (seg !== 7'h19 || dp !== 0) $fatal(1, "System display digit 0");
            wait (an == 4'hd); #1;
            if (seg !== 7'h30 || dp !== 1) $fatal(1, "System display digit 1");
            wait (an == 4'hb); #1;
            if (seg !== 7'h24 || dp !== 0) $fatal(1, "System display digit 2");
            wait (an == 4'h7); #1;
            if (seg !== 7'h79 || dp !== 1) $fatal(1, "System display digit 3");
        end
    join
    system_checks_done = 1;
end
// The enclosing bench must finish the integration exercise before resetting.
always @(posedge rst) begin
    if ($time > 1000 && !system_checks_done)
        $fatal(1, "System output integration did not complete");
end
