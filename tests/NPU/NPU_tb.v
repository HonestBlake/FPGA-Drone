`include "../../Drone/NPU/NPU.v"
`timescale 1ns / 1ps
module NPU_tb;
    reg clk = 0, rst = 1, healthy=1, pixel_valid=0;
    reg [5:0] address=0;
    reg [15:0] pixel=0;
    wire available;
    wire action_valid;
    wire [2:0] action;
    `include "vector_count.vh"
    reg [15:0] vectors [0:VECTOR_COUNT*64-1];
    reg signed [31:0] expected_scores [0:VECTOR_COUNT*5-1];
    reg [2:0] expected_class [0:VECTOR_COUNT-1];
    integer cycle, frame, index, pulses=0;
    always #5 clk = ~clk;
    NPU #(.ENABLE(1),.WEIGHTS_FILE("../../Drone/NPU/weights.mem"),
        .BIASES_FILE("../../Drone/NPU/biases.mem")) dut (
        .clk(clk),.rst(rst),.healthy(healthy),.pixel_valid(pixel_valid),
        .pixel_address(address),.pixel_mm(pixel),.available(available),.action_valid(action_valid),.action(action));
    wire disabled_available, disabled_valid;
    NPU disabled(.clk(clk),.rst(rst),.available(disabled_available),.action_valid(disabled_valid));
    always @(negedge clk) if(action_valid) pulses=pulses+1;
    task put;
        input [5:0] a;
        input [15:0] value;
        begin @(negedge clk);pixel_valid=1;address=a;pixel=value;end
    endtask
    initial begin
        $dumpfile("NPU.vcd"); $dumpvars(0, NPU_tb);
        `include "model_vectors.vh"
        repeat(3) @(negedge clk);rst=0;
        if(available || action_valid || disabled_available || disabled_valid) $fatal(1,"Startup validity");
        // Partial and out-of-order frames must not trigger inference.
        put(0,1000);put(2,1000);put(63,1000);
        @(negedge clk);pixel_valid=0;repeat(80) @(negedge clk);
        if(available || pulses) $fatal(1,"Partial frame classified");
        for(frame=0;frame<VECTOR_COUNT;frame=frame+1) begin
            for(index=0;index<64;index=index+1) put(index,vectors[frame*64+index]);
            @(negedge clk);pixel_valid=0;
            wait(action_valid);#1;
            if(!available || action!==expected_class[frame]) $fatal(1,"Reference class frame %0d",frame);
            for(index=0;index<5;index=index+1)
                if(dut.inference.scores[index]!==expected_scores[frame*5+index])
                    $fatal(1,"Reference score frame %0d neuron %0d",frame,index);
            repeat(4) @(negedge clk);
            if(action_valid) $fatal(1,"Result pulse held high");
        end
        if(pulses!=VECTOR_COUNT) $fatal(1,"Repeated frame result count %0d",pulses);
        healthy=0;#1;if(available || action_valid) $fatal(1,"Unhealthy output not gated");
        repeat(3) @(negedge clk);healthy=1;
        repeat(80) @(negedge clk);if(available) $fatal(1,"Old prediction resurrected");
        for(index=0;index<64;index=index+1) put(index,1000);
        @(negedge clk);pixel_valid=0;repeat(10) @(negedge clk);rst=1;
        repeat(3) @(negedge clk);rst=0;repeat(80) @(negedge clk);
        if(available || action_valid) $fatal(1,"Reset did not abort inference");
        $display("PASS: NPU actual weights, five classes/scores, signed input extremes, repeated/partial frames, health, reset and disabled default");
        $finish;
    end
    initial begin #1000000;$fatal(1,"NPU watchdog");end
endmodule
