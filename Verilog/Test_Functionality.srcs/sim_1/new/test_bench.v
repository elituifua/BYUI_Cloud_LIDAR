`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/04/2024 02:01:38 PM
// Design Name: 
// Module Name: test_bench
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


module test_bench(
    );
reg clk_n, clk_p;
wire [0:0] LED;
wire [4:0] JC;
wire [31:0] COUNT_OUT;
wire ENABLE_OUT, RST_OUT;

Test_1 dut (.sysclk_n(clk_n), .sysclk_p(clk_p), .jc(JC), .led(LED), .COUNT_TEST(COUNT_OUT), .ENABLE(ENABLE_OUT), .RESET(RST_OUT));

initial
begin
clk_n = 1'b0;
forever #2 clk_n = ~clk_n;
end

initial
begin
clk_p = 1'b1;
forever #2 clk_p = ~clk_p;
end

initial
begin

#1000000000;
$finish;
end


endmodule
