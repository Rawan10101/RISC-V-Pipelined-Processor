`timescale 1ns / 1ps
/*******************************************************************
*
* Project: RISC V Single Cycle Datapath
* Author1: Rawan Muhammad
* Author2: Salma El-Hawary 
* Author1-Email: Rawan_Khalid@aucegypt.edu
* Author2-Email: salma_el-hawary@aucegypt.edu
**********************************************************************/
module ALU_control_unit(
    input [1:0] ALUOp,
    input [2:0] inst14to12,      
    input inst30,                 
    input [6:0] inst31to25,       
    input R_type,                 
    output reg [4:0] ALUsel       
);

    always @(*) begin
        case (ALUOp)
            2'b00: ALUsel = 5'b00010; // LW, SW -> ADD
            2'b01: ALUsel = 5'b00110; // BEQ/BNE -> SUB
            2'b10: begin
                if (R_type) begin
                    // Check for M-extension (funct7 = 0000001)
                    if (inst31to25 == 7'b0000001) begin
                        case (inst14to12)
                            3'b000: ALUsel = 5'b01010; // MUL
                            3'b001: ALUsel = 5'b01011; // MULH
                            3'b010: ALUsel = 5'b01100; // MULHSU
                            3'b011: ALUsel = 5'b01101; // MULHU
                            3'b100: ALUsel = 5'b01110; // DIV
                            3'b101: ALUsel = 5'b01111; // DIVU
                            3'b110: ALUsel = 5'b10000; // REM
                            3'b111: ALUsel = 5'b10001; // REMU
                            default: ALUsel = 5'b00000;
                        endcase
                    end else begin
                        // Standard R-type instructions
                        case (inst14to12)
                            3'b000: ALUsel = inst30 ? 5'b00110 : 5'b00010; // SUB / ADD
                            3'b111: ALUsel = 5'b00000; // AND
                            3'b110: ALUsel = 5'b00001; // OR
                            3'b001: ALUsel = 5'b00101; // SLL
                            3'b010: ALUsel = 5'b00011; // SLT
                            3'b011: ALUsel = 5'b00100; // SLTU
                            3'b100: ALUsel = 5'b00111; // XOR
                            3'b101: ALUsel = inst30 ? 5'b01001 : 5'b01000; // SRA / SRL
                            default: ALUsel = 5'b00000;
                        endcase
                    end
                end else begin
                    // I-type arithmetic instructions
                    case (inst14to12)
                        3'b000: ALUsel = 5'b00010; // ADDI
                        3'b111: ALUsel = 5'b00000; // ANDI
                        3'b110: ALUsel = 5'b00001; // ORI
                        3'b001: ALUsel = 5'b00101; // SLLI
                        3'b010: ALUsel = 5'b00011; // SLTI
                        3'b011: ALUsel = 5'b00100; // SLTIU
                        3'b100: ALUsel = 5'b00111; // XORI
                        3'b101: ALUsel = inst30 ? 5'b01001 : 5'b01000; // SRAI / SRLI
                        default: ALUsel = 5'b00000;
                    endcase
                end
            end
            default: ALUsel = 5'b00000;
        endcase
    end
endmodule
