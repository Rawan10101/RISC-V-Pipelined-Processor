`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/30/2025 04:48:14 PM
// Design Name: 
// Module Name: EXP4
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
module immediate_generator(
    output reg [31:0] gen_out,
    input  [31:0] instr
);
    wire [6:0] opcode = instr[6:0];
    wire [2:0] funct3 = instr[14:12];
    wire       funct7b5 = instr[30]; // bit 30 distinguishes SRLI / SRAI

    always @(*) begin
        case(opcode)
            // I-type: ADDI, ANDI, ORI, LW, JALR, SLLI, SRLI, SRAI
            7'b0010011, // I-type ALU
            7'b0000011, // Load
            7'b1100111: // JALR
                begin
                    if (funct3 == 3'b001 || funct3 == 3'b101) begin
                        gen_out = {27'b0, instr[24:20]}; // SLLI, SRLI, SRAI -> shamt
                    end else begin
                        gen_out = {{20{instr[31]}}, instr[31:20]}; // normal I-type imm
                    end
                end

            // S-type: Store (SW, SH, SB)
            7'b0100011:
                gen_out = {{20{instr[31]}}, instr[31:25], instr[11:7]};

            // B-type: Branches (BEQ, BNE, etc.)
            7'b1100011:
                gen_out = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};

            // U-type: LUI, AUIPC
            7'b0110111, // LUI
            7'b0010111: // AUIPC
                gen_out = {instr[31:12], 12'b0};

            // J-type: JAL
            7'b1101111:
                gen_out = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};

            default:
                gen_out = 32'b0;
        endcase
    end
 
endmodule

