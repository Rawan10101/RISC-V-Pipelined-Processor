`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/17/2025 01:16:52 PM
// Design Name: 
// Module Name: HazardDetectionUnit
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

module HazardDetectionUnit(
    input ID_EX_MemRead,
    input [4:0] ID_EX_Rd,
    input [4:0] IF_ID_Rs1,
    input [4:0] IF_ID_Rs2,
    output reg PCWrite,
    output reg IF_ID_Write,
    output reg ID_EX_Bubble
);
    always @(*) begin
        PCWrite       = 1'b1;
        IF_ID_Write   = 1'b1;
        ID_EX_Bubble  = 1'b0;

        if (ID_EX_MemRead &&
           ((ID_EX_Rd == IF_ID_Rs1) || (ID_EX_Rd == IF_ID_Rs2))) begin
            PCWrite      = 1'b0;
            IF_ID_Write  = 1'b0;
            ID_EX_Bubble = 1'b1;
        end
    end
endmodule
