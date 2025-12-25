`timescale 1ns / 1ps
/*******************************************************************
*
* Project: RISC V Single Cycle Datapath
* Author1: Rawan Muhammad
* Author2: Salma El-Hawary 
* Author1-Email: Rawan_Khalid@aucegypt.edu
* Author2-Email: salma_el-hawary@aucegypt.edu
**********************************************************************/
module _32_bit_ALU #(
    parameter N = 32
)(
    input  [N-1:0] A,
    input  [N-1:0] B,          // rs2 or immediate
    input  [4:0]   shamt,      // immediate shift amount 
    input  [4:0]   sel,        // ALU op
    output reg [N-1:0] ALU_result,
    output reg zero_flag,
    output reg carry_flag,
    output reg overflow_flag,
    output reg sign_flag
);

    // ADD / SUB logic
    wire [N-1:0] B_effective = (sel == 4'b0110) ? (~B + 1'b1) : B; // SUB
    wire [N-1:0] sum;
    wire carry_out;

    rca #(N) rca1 (
        .a(A),
        .b(B_effective),
        .sum(sum),
        .carry_out(carry_out)
    );

    // SHIFT 
        wire [4:0] shift_amt = (sel == 4'b0101 || sel == 4'b1000 || sel == 4'b1001)
                           ? B[4:0]   // R-type shifts use rs2[4:0]
                           : shamt;   // I-type shifts use immediate

    wire [N-1:0] SLL = A << shift_amt;
    wire [N-1:0] SRL = A >> shift_amt;
    wire [N-1:0] SRA = $signed(A) >>> shift_amt;

wire signed [31:0] A_signed = $signed(A);
wire signed [31:0] B_signed = $signed(B);
wire [31:0] A_unsigned = A;
wire [31:0] B_unsigned = B;

 // M-extension: Multiplication and Division
    wire signed [63:0] mul_result_signed = A_signed * B_signed;
    wire [63:0] mul_result_unsigned = A_unsigned * B_unsigned; //mulhu
    wire signed [63:0] mul_result_su = A_signed * $signed({32'h00000000, B_unsigned});
    
    wire signed [N-1:0] div_result_signed = A_signed / B_signed;
    wire [N-1:0] div_result_unsigned = A / B;
wire signed [N-1:0] rem_result_signed   = (B_signed == 0) ? A_signed : (A_signed % B_signed);
wire [N-1:0]        rem_result_unsigned = (B_unsigned == 0) ? A_unsigned : (A_unsigned % B_unsigned);


    always @(*) begin
        
        ALU_result    = {N{1'b0}};
        zero_flag     = 0;
        carry_flag    = 0;
        overflow_flag = 0;
        sign_flag     = 0;

        case(sel)
            
            5'b00010: ALU_result = sum;                         // ADD / ADDI
            5'b00110: ALU_result = sum;                         // SUB
            5'b00000: ALU_result = A & B;                       // AND / ANDI
            5'b00001: ALU_result = A | B;                       // OR / ORI
            5'b00111: ALU_result = A ^ B;                       // XOR / XORI
            5'b00011: ALU_result = ($signed(A) < $signed(B)) ? 32'b1 : 32'b0; // SLT / SLTI
            5'b00100: ALU_result = (A < B) ? 32'b1 : 32'b0;     // SLTU / SLTIU
            5'b00101: ALU_result = SLL;                         // SLL / SLLI
            5'b01000: ALU_result = SRL;                         // SRL / SRLI
            5'b01001: ALU_result = SRA;                         // SRA / SRAI
         
         
         // M-extension instructions
            5'b01010: ALU_result = mul_result_signed[31:0];     // MUL (lower 32 bits)
            5'b01011: ALU_result = mul_result_signed[63:32];    // MULH (upper 32 bits, signed x signed)
            5'b01100: ALU_result = mul_result_su[63:32];        // MULHSU (upper 32 bits, signed x unsigned)
            5'b01101: ALU_result = mul_result_unsigned[63:32];  // MULHU (upper 32 bits, unsigned x unsigned)
            5'b01110: ALU_result = div_result_signed;           // DIV (signed)
            5'b01111: ALU_result = div_result_unsigned;         // DIVU (unsigned)
            5'b10000: ALU_result = rem_result_signed;          // REM
            5'b10001: ALU_result = rem_result_unsigned;        // REMU
            default: ALU_result = {N{1'b0}};
        endcase

        zero_flag = ~|ALU_result;
        sign_flag = ALU_result[N-1];

        case(sel)
            4'b0010, 4'b0110: carry_flag = carry_out;
            default: carry_flag = 0;
        endcase

        case(sel)
            5'b00010: overflow_flag = (A[N-1] == B[N-1]) && (ALU_result[N-1] != A[N-1]); // ADD
            5'b00110: overflow_flag = (A[N-1] != B[N-1]) && (ALU_result[N-1] != A[N-1]); // SUB
            default: overflow_flag = 0;
        endcase
    end
endmodule
