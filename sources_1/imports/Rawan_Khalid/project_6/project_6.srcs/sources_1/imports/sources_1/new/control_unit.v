`timescale 1ns / 1ps
/*******************************************************************
*
* Project: RISC V Single Cycle Datapath
* Author1: Rawan Muhammad
* Author2: Salma El-Hawary 
* Author1-Email: Rawan_Khalid@aucegypt.edu
* Author2-Email: salma_el-hawary@aucegypt.edu
**********************************************************************/
module control_unit ( 
    input  [4:0] inst,          // opcode[6:2]
    input  [2:0] funct3,        // instruction[14:12]
    output reg branch,
    output reg MemRead,
    output reg MemtoReg,
    output reg [1:0] ALUOP,
    output reg MemWrite,
    output reg ALUSrc,
    output reg RegWrite,
    output reg [1:0] size,      // 00=byte, 01=halfword, 10=word
    output reg signed_op,       // 1=signed, 0=unsigned
    output reg LUI,
    output reg AUIPC,
    output reg JAL,
    output reg JALR
);

always @(*) begin
    branch   = 0;
    MemRead  = 0;
    MemtoReg = 0;
    ALUOP    = 2'b00;
    MemWrite = 0;
    ALUSrc   = 0;
    RegWrite = 0;
    size     = 2'b10;  
    signed_op= 1'b1;

    LUI   = 0;
    AUIPC = 0;
    JAL   = 0;
    JALR  = 0;

    case(inst)
        5'b01100: begin // R-type //M instructions same op
            ALUOP    = 2'b10;
            ALUSrc   = 0;
            RegWrite = 1;
        end

        5'b00100: begin // I-type arithmetic
            ALUOP    = 2'b10;
            ALUSrc   = 1;
            RegWrite = 1;
        end

        5'b00000: begin // Load
            ALUOP    = 2'b00;
            ALUSrc   = 1;
            RegWrite = 1;
            MemRead  = 1;
            MemtoReg = 1;
             case(funct3)
    3'b000: begin size = 2'b00; signed_op = 1'b1; end // LB
    3'b001: begin size = 2'b01; signed_op = 1'b1; end // LH
    3'b010: begin size = 2'b10; signed_op = 1'b1; end // LW
    3'b100: begin size = 2'b00; signed_op = 1'b0; end // LBU
    3'b101: begin size = 2'b01; signed_op = 1'b0; end // LHU
endcase
        end

        5'b01000: begin // Store
            ALUOP    = 2'b00;
            ALUSrc   = 1;
            RegWrite = 0;
            MemWrite = 1;
            case(funct3)
                3'b000: size=2'b00; // SB
                3'b001: size=2'b01; // SH
                3'b010: size=2'b10; // SW
            endcase
        end

        5'b11000: begin // Branch
            ALUOP    = 2'b01;
            ALUSrc   = 0;
            RegWrite = 0;
            branch   = 1;
        end

        5'b01101: begin // LUI
            ALUOP    = 2'b00;
            ALUSrc   = 1;
            RegWrite = 1;
            LUI      = 1;
        end

        5'b00101: begin // AUIPC
            ALUOP    = 2'b00;
            ALUSrc   = 1;
            RegWrite = 1;
            AUIPC    = 1;
        end

        5'b11011: begin // JAL
            ALUOP    = 2'b00;
            ALUSrc   = 1;
            RegWrite = 1;
            branch   = 1;
            JAL      = 1;
        end

        5'b11001: begin // JALR
            ALUOP    = 2'b00;
            ALUSrc   = 1;
            RegWrite = 1;
            branch   = 1;
            JALR     = 1;
            end
        5'b1110: begin  //systemInstructions
            ALUOP    = 2'b00;
            ALUSrc   = 0;
            RegWrite = 0;
            branch   = 0;
            JALR     = 0;
        end
        
    endcase
end
endmodule
