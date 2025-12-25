`timescale 1ns / 1ps
/*******************************************************************
*
* Project: RISC V Single Cycle Datapath
* Author1: Rawan Muhammad
* Author2: Salma El-Hawary 
* Author1-Email: Rawan_Khalid@aucegypt.edu
* Author2-Email: salma_el-hawary@aucegypt.edu
**********************************************************************/
module branch_decision(
    input        branch,      // 1 if instruction is a branch
    input  [2:0] funct3,      
    input        Z,           
    input        S,          
    input        V,           
    input        C,          
    output reg   PCSrc        // 1 = take branch
);

    always @(*) begin
        if(branch) begin
            case(funct3)
                3'b000: PCSrc = Z;           // BEQ
                3'b001: PCSrc = ~Z;          // BNE
                3'b100: PCSrc = (S != V);    // BLT
                3'b101: PCSrc = (S == V);    // BGE
                3'b110: PCSrc = ~C;          // BLTU
                3'b111: PCSrc = C;           // BGEU
                default: PCSrc = 0;
            endcase
        end else begin
            PCSrc = 0; // not a branch
        end
    end

endmodule
