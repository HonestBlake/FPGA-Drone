`ifndef TEST_XADC_MODEL
`define TEST_XADC_MODEL
`timescale 1ns / 1ps
// Digital protocol test double only. Not an analog or UNISIM model.
// Benches set sample_code to emulate an ADC conversion.
module XADC #(
    parameter [15:0] INIT_40 = 0, INIT_41 = 0, INIT_42 = 0
) (
    input DCLK, RESET, DEN, DWE, CONVST, CONVSTCLK, VP, VN,
    input [6:0] DADDR,
    input [15:0] DI, VAUXP, VAUXN,
    output reg [15:0] DO,
    output reg DRDY, EOC,
    output EOS, BUSY, OT, JTAGBUSY, JTAGLOCKED, JTAGMODIFIED,
    output [4:0] CHANNEL, MUXADDR,
    output [7:0] ALM
);
    reg [11:0] sample_code = 0;
    reg [3:0] phase;
    reg pending;
    reg [15:0] pending_data;
    assign CHANNEL = INIT_40[4:0];
    assign {EOS, BUSY, OT, JTAGBUSY, JTAGLOCKED, JTAGMODIFIED, MUXADDR, ALM} = 0;
    always @(posedge DCLK) begin
        if (RESET) begin
            phase <= 0; EOC <= 0; DRDY <= 0; DO <= 0;
            pending <= 0; pending_data <= 0;
        end else begin
            phase <= phase + 1'b1;
            EOC <= (phase == 12);
            DRDY <= pending;
            pending <= DEN;
            if (DEN) begin
                if (DADDR !== {2'b0, INIT_40[4:0]} || DWE !== 0)
                    $fatal(1, "XADC model: bad DRP address/write");
                pending_data <= {sample_code, 4'b0};
            end
            if (pending) DO <= pending_data;
        end
    end
endmodule
`endif
