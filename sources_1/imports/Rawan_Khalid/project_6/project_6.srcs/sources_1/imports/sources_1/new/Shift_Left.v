`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/30/2025 04:29:13 PM
// Design Name: 
// Module Name: EXP3
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


module Shift_Left #(
    parameter N = 8
)(
    input  [N-1:0] d,
    output [N-1:0] y
);

    assign y[0] = 1'b0; 

    genvar i;
    generate
        for (i = 1; i < N; i = i + 1) begin : loop
            assign y[i] = d[i-1];
        end
    endgenerate

endmodule
