`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/17/2025 01:15:18 PM
// Design Name: 
// Module Name: ForwardingUnit
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
module ForwardingUnit(
    input EX_MEM_RegWrite,
    input MEM_WB_RegWr,
    input [4:0] EX_MEM_Rd,
    input [4:0] MEM_WB_Rd,
    input [4:0] ID_EX_Rs1,
    input [4:0] ID_EX_Rs2,
    output reg [1:0] ForwardA,
    output reg [1:0] ForwardB
);
    always @(*) begin
        ForwardA = 2'b00;
        ForwardB = 2'b00;

        // EX 
        if (EX_MEM_RegWrite && (EX_MEM_Rd != 0) && (EX_MEM_Rd == ID_EX_Rs1))
            ForwardA = 2'b10;
        if (EX_MEM_RegWrite && (EX_MEM_Rd != 0) && (EX_MEM_Rd == ID_EX_Rs2))
            ForwardB = 2'b10;

        // MEM 
        if (MEM_WB_RegWr && (MEM_WB_Rd != 0) &&
            !(EX_MEM_RegWrite && (EX_MEM_Rd != 0) && (EX_MEM_Rd == ID_EX_Rs1)) &&
            (MEM_WB_Rd == ID_EX_Rs1))
            ForwardA = 2'b01;
        if (MEM_WB_RegWr && (MEM_WB_Rd != 0) &&
            !(EX_MEM_RegWrite && (EX_MEM_Rd != 0) && (EX_MEM_Rd == ID_EX_Rs2)) &&
            (MEM_WB_Rd == ID_EX_Rs2))
            ForwardB = 2'b01;
    end
endmodule
