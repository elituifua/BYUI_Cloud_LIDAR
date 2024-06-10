`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/15/2024 05:10:06 PM
// Design Name: 
// Module Name: Test_1
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

// Test Plan:
    // Test the eval board using several different delay times between the the start and stop inputs
        // Could we get an LED with the right frequency to simulate the returning laser?
        // We need to set up a receiving component to process the start signal from the microcontroller



// set up a clock module to have 200 MHz
    //Consider setting up multiple clocks at 20 MHz, 2 MHz, etc for potentially better accuracy
// setup an output module with a counter
    // set up counter to be easily changed, maybe even use the basys3 and make it programmable with the input buttons
    //
    
    
    
    
module Test_1(input sysclk_n, sysclk_p, output [4:0]jc, [0:0]led);
//module Test_1(input sysclk_n, sysclk_p, output [4:0]jc, [0:0]led, [32:0] COUNT_TEST, RESET, ENABLE);

wire enable;
wire stop;
wire sysclk;
wire slw_clk;
//assign RESET = stop;
//assign ENABLE = enable;

clk_wiz_0 CLOCK1 (.clk_out1(sysclk), .clk_in1_p(sysclk_p), .clk_in1_n(sysclk_n));
control ON_OFF (.rst(stop), .clock(slw_clk), .out(enable));
slow_clock CLK_SLOW (.clk_in(sysclk), .clk_out(slw_clk));
counter COUNT(.clk(sysclk), .enable(enable), .turn_off(stop), .out1(jc[0]), .out2(jc[1]));
//counter COUNT(.clk(sysclk), .enable(enable), .turn_off(stop), .out1(jc[0]), .out2(jc[1]), .COUNT(COUNT_TEST));
assign led[0] = slw_clk;
assign jc[2] = slw_clk;
assign jc[3] = 1'b1;
assign jc[4] = sysclk;
endmodule


module slow_clock(input clk_in, output clk_out); // Slow the clock down from 200 MHz to ~0.37 Hz

reg [32:0] count;
initial count = 8'h00000000;


always @ (posedge clk_in)
begin
count = count + 1;
end

assign clk_out = count[29]; // Should result in a period of about 5.3 seconds
//assign clk_out = count[19]; // For testing

endmodule



// Anytime the count finishes a test, it will send a reset signal to this module which will turn off the enable signal which
// will reset everything until the slow clock turns it back on again to allow for another test to be performed.
module control(input rst, input clock, output reg out); 

initial out = 1'b0;

always @ (posedge rst, posedge clock)
begin
if (rst)
    begin
        out = 1'b0;
    end
 else
    begin
        out = 1'b1;
    end
end

endmodule



// Keeps track of the number of clock cycles that has passed.
module counter(clk, enable, turn_off, out1, out2);
//module counter(clk, enable, turn_off, out1, out2, COUNT);
    input clk, enable;
    output reg out1, out2, turn_off;
//    output [32:0] COUNT;

    reg [32:0] count;
//    assign COUNT = count;
   
    initial
    begin
    count = 8'h00000000;
    out1 = 1'b0;
    out2 = 1'b0;
    turn_off = 1'b0;
    end
    
    
    always @(posedge clk)   // as clk goes high, if enable is high, add 1 to counter
    begin
    if (enable == 1'b1)
        begin
        count = count + 1;
        if (count == 32'h0800000f) turn_off = 1'b1;
        //if (count == 32'h0002000f) turn_off = 1'b1; // For testing
        end
    else
        begin
        count = 10'b0000000000;
        turn_off = 1'b0;
        end
    end
    
    always @(count)
    begin
        case(count)
        32'h00000040: out1 = 1'b1;    // send first signal at 64 clock cycles
        32'h000000e0: out2 = 1'b1;    // The number of clock cycles that it will take till sending the second signal (224-64=160)
        32'h08000000:                 // Reset outputs at _ clock cycles
        //32'h00020000:                 // for testing
            begin
            out1 = 1'b0;
            out2 = 1'b0;
            end
        endcase 
    end

endmodule

