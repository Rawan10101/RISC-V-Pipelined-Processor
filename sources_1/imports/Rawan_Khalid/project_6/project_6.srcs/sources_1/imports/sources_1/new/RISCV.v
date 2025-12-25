`timescale 1ns / 1ps
/*******************************************************************
*
* Project: RISC V Single Cycle Datapath
* Author1: Rawan Muhammad
* Author2: Salma El-Hawary
* Author1-Email: Rawan_Khalid@aucegypt.edu
* Author2-Email: salma_el-hawary@aucegypt.edu
* Description: RISC_V top module
**********************************************************************/
module RISC_V (
    input clk,
    input rst,
    input [1:0] ledSel,
    input [4:0] ssdSel,
    output reg [15:0] leds,
    output reg [12:0] ssd
);
reg sclk =0;
wire nsclk=~sclk;

always @ (posedge clk or posedge rst) begin
if (rst) sclk<=1'b0;
else sclk <=~sclk;
end
wire [31:0] PC;
    wire [31:0] nextPC;

N_bit_regsiter #(32) PC_reg_inst (
    .clk(sclk),
    .rst(rst),
    .load(1'b1),   // PCWrite controls whether to update PC
    .data_in(nextPC),
    .data_out(PC)
);

//always @(posedge sclk or posedge rst) begin
//    if (rst) begin
//        PC_reg <= 32'b0;
//    end else begin
//        if (PCWrite)
//            PC_reg <= nextPC;
//        else
//            PC_reg <= PC_reg; // hold (explicit for clarity)
//    end
//end

// IF Stage
      wire PCWrite, IF_ID_Write_Flush, ID_EX_Bubble;
//    reg [31:0] PC_reg;
//    wire [31:0] PC = PC_reg;
    wire [31:0] PCplus4 = PC + 4;

    wire [31:0] IF_Inst;
    wire sel_data;
    assign sel_data= (EX_MEM_Ctrl[5] || EX_MEM_Ctrl[4]);
   wire [7:0] instr_addr;
assign instr_addr = PC[7:0];  

wire [7:0] data_addr;
assign data_addr = EX_MEM_ALU_out[7:0];




    // sclk = half-speed clock
wire [7:0] mem_addr;
wire mem_write_en, mem_read_en;
assign mem_addr = (sclk) ? instr_addr :data_addr ;
assign mem_write_en = sclk ?1'b0: EX_MEM_Ctrl[4];  // only write during sclk=1
assign mem_read_en  = sclk ? 1'b1 : EX_MEM_Ctrl[5];  // only read during sclk=1
 wire [2:0] mem_inst = sclk ? 3'b010 :  EX_MEM_funct3 ;// default to LW type
//wire is_unsigned_mem = ~signed_op; // 1 for LBU/LHU, 0 for LB/LH
wire [31:0] mem_datain;
wire [31:0] memOut;

assign mem_datain=sclk?32'b0: EX_MEM_RegR2;
    unified_mem U_MEM(.sclk(nsclk),.rst(rst),.addr(mem_addr),.write_data(EX_MEM_RegR2), .mem_write(mem_write_en),.inst(mem_inst),.MemRead(mem_read_en), .read_data(memOut));
 
    
    reg [31:0] IF_Inst_reg;

always @(posedge nsclk or posedge rst) begin
    if (rst)
        IF_Inst_reg <= 32'b0;
    else
        IF_Inst_reg <= memOut;   // instr cycle only
end

assign IF_Inst = IF_Inst_reg;

//    // IF/ID Pipeline Register
//    wire [31:0] IF_ID_PC, IF_ID_Inst;
//  wire flush_IF_ID;
//assign flush_IF_ID = EX_MEM_BranchTaken || JAL || EX_MEM_JALR;

//// Insert NOP (all zeros) when flushing
//wire [31:0] IF_Inst_flushed;
//assign IF_Inst_flushed = flush_IF_ID ? 32'h00000013 : IF_Inst;  // NOP = ADDI x0, x0, 0

//// IF/ID Pipeline Register with flush module _32_bit_ALU


wire flush_IF_ID_immediate;
// Flush when branch taken in EX stage (immediate) or JAL in ID stage
assign flush_IF_ID_immediate = PCSrc_ex || JAL || ID_EX_JALR;

// Insert NOP when flushing
wire [31:0] IF_Inst_flushed;
assign IF_Inst_flushed = flush_IF_ID_immediate ? 32'h00000013 : IF_Inst;

// IF/ID Pipeline Register with immediate flush


wire [31:0] IF_ID_PC, IF_ID_Inst;

N_bit_regsiter #(64) IF_ID (
    .clk(sclk),
    .rst(rst),
    .load(1'b1),
    .data_in({PC, IF_Inst_flushed}),
    .data_out({IF_ID_PC, IF_ID_Inst})
);


    // ID Stage
    wire [4:0] opcode = IF_ID_Inst[6:2];
    wire [2:0] funct3 = IF_ID_Inst[14:12];
    wire funct7 = IF_ID_Inst[30];
    wire [6:0] funct7_full = IF_ID_Inst[31:25];  // CHANGED: Extract full funct7
    wire [4:0] rs1 = IF_ID_Inst[19:15];
    wire [4:0] rs2 = IF_ID_Inst[24:20];
    wire [4:0] rd  = IF_ID_Inst[11:7];
    wire R_type = (IF_ID_Inst[6:0] == 7'b0110011);

    // Control signals
    wire branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite;
   
    wire [1:0] ALUOp;
    wire [1:0] size;
    wire signed_op;
    wire LUI, AUIPC, JAL, JALR;

    control_unit CU (
        .inst(opcode),
        .funct3(funct3),
        .branch(branch),
        .MemRead(MemRead),
        .MemtoReg(MemtoReg),
        .ALUOP(ALUOp),
        .MemWrite(MemWrite),
        .ALUSrc(ALUSrc),
        .RegWrite(RegWrite),
        .size(size),
        .signed_op(signed_op),
        .LUI(LUI),
        .AUIPC(AUIPC),
        .JAL(JAL),
        .JALR(JALR)
    );

 
//HazardDetectionUnit HDU(
//    .ID_EX_MemRead(ID_EX_Ctrl[5]),
//    .ID_EX_Rd(ID_EX_Rd),
//    .IF_ID_Rs1(rs1),
//    .IF_ID_Rs2(rs2),
//    .PCWrite(PCWrite),
//    .IF_ID_Write(IF_ID_Write_Flush),
//    .ID_EX_Bubble(ID_EX_Bubble)
//);



wire [8:0] ID_Ctrl = {R_type, RegWrite, MemtoReg, MemRead, MemWrite, branch, ALUSrc, ALUOp};

wire [8:0] flushallInstrl;
assign flushallInstrl= (ID_EX_Bubble)? ( 9'b0): (ID_Ctrl);

    // Register file
    wire [31:0] regRead1, regRead2;
    wire [31:0] regWriteData;

    register_file RF (
        .readreg1(rs1),
        .readreg2(rs2),
        .writereg(MEM_WB_Rd),
        .writedata(regWriteData),
        .reset(rst),
        .clk(sclk),
        .regWrite(MEM_WB_Ctrl[7]),  // RegWrite at MEM/WB
        .readdata1(regRead1),
        .readdata2(regRead2)
    );

    // Immediate generation
    wire [31:0] immGenOut;
    immediate_generator IMM(.instr(IF_ID_Inst), .gen_out(immGenOut));

    // ID/EX pipeline register
    wire [8:0]  ID_EX_Ctrl;
    wire [31:0] ID_EX_PC, ID_EX_RegR1, ID_EX_RegR2, ID_EX_Imm;
    wire [3:0]  ID_EX_Func;
    
    wire [4:0]  ID_EX_Rs1, ID_EX_Rs2, ID_EX_Rd;
   wire        ID_EX_JAL, ID_EX_JALR;
   
//  N_bit_regsiter #(158) ID_EX ( // increased width: previous 156 + 2 bits for JAL/JALR
//    .clk(clk), .rst(rst), .load(1'b1),
//    .data_in({flushallInstrl, IF_ID_PC, regRead1, regRead2, immGenOut,
//              {funct7, funct3}, rs1, rs2, rd, JAL, JALR}), // attach JAL/JALR as LSBs
//    .data_out({ID_EX_Ctrl, ID_EX_PC, ID_EX_RegR1, ID_EX_RegR2,
//               ID_EX_Imm, ID_EX_Func, ID_EX_Rs1, ID_EX_Rs2, ID_EX_Rd,
//               ID_EX_JAL, ID_EX_JALR})
//);

wire ID_EX_LUI, ID_EX_AUIPC;
wire [6:0]funct7_full_ID_EX;



wire flush_ID_EX_immediate;
assign flush_ID_EX_immediate = PCSrc_ex || ID_EX_JALR;

wire [8:0] ID_Ctrl_flushed;
assign ID_Ctrl_flushed = flush_ID_EX_immediate ? 9'b0 : ID_Ctrl;


N_bit_regsiter #(167) ID_Ex ( // increased width by 2 for LUI and AUIPC
    .clk(sclk), .rst(rst), .load(1'b1),
    .data_in({ID_Ctrl_flushed, IF_ID_PC, regRead1, regRead2, immGenOut,funct7_full,
              {funct7, funct3}, rs1, rs2, rd, LUI, AUIPC, JAL, JALR}),
    .data_out({ID_EX_Ctrl, ID_EX_PC, ID_EX_RegR1, ID_EX_RegR2,
               ID_EX_Imm, funct7_full_ID_EX, ID_EX_Func, ID_EX_Rs1, ID_EX_Rs2, ID_EX_Rd,
               ID_EX_LUI, ID_EX_AUIPC, ID_EX_JAL, ID_EX_JALR})
);



wire ID_EX_Rtype = ID_EX_Ctrl[8]; // msb we added

//    // EX Stage
//    wire [3:0] ALUsel;
//    ALU_control_unit ALU_CTRL (
//        .ALUOp(ID_EX_Ctrl[1:0]),
//        .inst14to12(ID_EX_Func[2:0]),
//        .inst30(ID_EX_Func[3]),
//        .R_type(ID_EX_Rtype),
//        .ALUsel(ALUsel)
//    );
    
    wire [4:0] ALUsel;
    ALU_control_unit ALU_CTRL (
        .ALUOp(ID_EX_Ctrl[1:0]),
        .inst14to12(ID_EX_Func[2:0]),
        .inst30(ID_EX_Func[3]),
        .inst31to25(funct7_full_ID_EX),
        .R_type(ID_EX_Rtype),
        .ALUsel(ALUsel)
    );

    
    
    
wire [1:0] ForwardA, ForwardB;

// Forwarding logic
ForwardingUnit FU (
    .EX_MEM_RegWrite(EX_MEM_Ctrl[7]), // correct
    .MEM_WB_RegWr(MEM_WB_Ctrl[7]),    // corrected
    .EX_MEM_Rd(EX_MEM_Rd),
    .MEM_WB_Rd(MEM_WB_Rd),
    .ID_EX_Rs1(ID_EX_Rs1),
    .ID_EX_Rs2(ID_EX_Rs2),
    .ForwardA(ForwardA),
    .ForwardB(ForwardB)
);

//// Forwarded ALU operands
//// Forwarded ALU operands
wire [31:0] ALUin1, ALUin2;

//// Operand A
//assign ALUin1 = (ForwardA == 2'b00) ? ID_EX_RegR1 :
//                (ForwardA == 2'b10) ? EX_MEM_ALU_out :
//                (ForwardA == 2'b01) ? regWriteData : ID_EX_RegR1;
wire [31:0] ALUin1_forwarded, ALUin2_base, ALUin2_forwarded;

// Forward operand A (Rs1)
assign ALUin1_forwarded = (ForwardA == 2'b10) ? EX_MEM_ALU_out :   // Forward from EX/MEM
                          (ForwardA == 2'b01) ? regWriteData :     // Forward from MEM/WB
                          ID_EX_RegR1;    
//// Operand B (32-bit)
wire [31:0] ALU_B_in_pre = (ID_EX_Ctrl[2]) ? ID_EX_Imm : ID_EX_RegR2;
assign ALUin2_base = (ID_EX_Ctrl[2]) ? ID_EX_Imm : ID_EX_RegR2;

// Forward operand B (Rs2) - but only forward register values, not immediates
wire [31:0] ID_EX_RegR2_forwarded;
assign ID_EX_RegR2_forwarded = (ForwardB == 2'b10) ? EX_MEM_ALU_out :  // Forward from EX/MEM
                                (ForwardB == 2'b01) ? regWriteData :     // Forward from MEM/WB
                                ID_EX_RegR2;                              // No forwarding

// Final ALU input B (apply ALUSrc after forwarding)
assign ALUin2_forwarded = (ID_EX_Ctrl[2]) ? ID_EX_Imm : ID_EX_RegR2_forwarded;

//assign ALUin2 = (ForwardB == 2'b00) ? ALU_B_in_pre :
//                (ForwardB == 2'b10) ? EX_MEM_ALU_out :
//                (ForwardB == 2'b01) ? regWriteData : ALU_B_in_pre;


    wire [31:0] aluSrcB = (ID_EX_Ctrl[2]) ? ID_EX_Imm : ID_EX_RegR2;

    wire [31:0] EX_ALU_out;
    wire zero_flag, carry_flag, overflow_flag, sign_flag;
   
_32_bit_ALU ALU (
    .A(ALUin1_forwarded),
    .B(ALUin2_forwarded),
    .shamt(5'b0),
    .sel(ALUsel),
    .ALU_result(EX_ALU_out),
    .zero_flag(zero_flag),
    .carry_flag(carry_flag),
    .overflow_flag(overflow_flag),
    .sign_flag(sign_flag)
);

//    _32_bit_ALU ALU (
//        .A(ID_EX_RegR1),
//        .B(aluSrcB),
//        .shamt(5'b0),
//        .sel(ALUsel),
//        .ALU_result(EX_ALU_out),
//        .zero_flag(zero_flag),
//        .carry_flag(carry_flag),
//        .overflow_flag(overflow_flag),
//        .sign_flag(sign_flag)
//    );
   
   wire [31:0] EX_ALU_result;

assign EX_ALU_result = (ID_EX_LUI)   ? ID_EX_Imm :           // LUI
                       (ID_EX_AUIPC) ? ID_EX_PC + ID_EX_Imm : // AUIPC
                       EX_ALU_out;                           // normal ALU ops
 

    wire [31:0] EX_BranchAddOut = ID_EX_PC + (ID_EX_Imm);
     wire PCSrc_ex;

    // for JALR we will use ALU output (EX_ALU_out) as jalr target (typical)
 branch_decision BD_EX (
        .branch(ID_EX_Ctrl[3]),        // branch bit from ID/EX control (instruction is a branch)
        .funct3(ID_EX_Func[2:0]),      // funct3 from ID/EX
        .Z(zero_flag),
        .S(sign_flag),
        .V(overflow_flag),
        .C(carry_flag),
        .PCSrc(PCSrc_ex)               // 1 if this branch should be taken
    );

wire [31:0] EX_RegR2_forwarded;
assign EX_RegR2_forwarded = (ForwardB == 2'b10) ? EX_MEM_ALU_out :
                             (ForwardB == 2'b01) ? regWriteData :
                             ID_EX_RegR2;
                             
    wire [7:0]  EX_Ctrl = ID_EX_Ctrl[7:0];
    wire [31:0] EX_RegR2 = EX_RegR2_forwarded;
    wire [4:0]  EX_Rd = ID_EX_Rd;
    wire EX_Zero = zero_flag;
    wire EX_BranchTaken_ex = PCSrc_ex;
    // EX/MEM pipeline register
 
        wire [7:0]  EX_MEM_Ctrl;
    wire [31:0] EX_MEM_BranchAddOut, EX_MEM_ALU_out, EX_MEM_RegR2;
    wire [4:0]  EX_MEM_Rd;
    wire        EX_MEM_BranchTaken; // stored branch-decision bit (1 = take branch)
    wire [2:0]  EX_MEM_funct3;
wire EX_MEM_LUI, EX_MEM_AUIPC;
wire [31:0] EX_MEM_PCplus4;
// EX stage: compute return address
wire [31:0] EX_PCplus4 = ID_EX_PC + 4;

wire EX_MEM_JALR,EX_MEM_JAL;

N_bit_regsiter #(149) EX_MEM (
    .clk(!sclk), .rst(rst), .load(1'b1),
    .data_in({EX_Ctrl, EX_BranchAddOut, EX_BranchTaken_ex, EX_ALU_result,
              EX_RegR2, EX_Rd, ID_EX_Func[2:0], ID_EX_LUI, ID_EX_AUIPC, ID_EX_JAL, ID_EX_JALR, EX_PCplus4}),
    .data_out({EX_MEM_Ctrl, EX_MEM_BranchAddOut, EX_MEM_BranchTaken,
               EX_MEM_ALU_out, EX_MEM_RegR2, EX_MEM_Rd, EX_MEM_funct3,
               EX_MEM_LUI, EX_MEM_AUIPC, EX_MEM_JAL, EX_MEM_JALR, EX_MEM_PCplus4})
);



    wire [31:0] memOut;

    wire [7:0] MEM_Ctrl = EX_MEM_Ctrl;
    wire [4:0] MEM_Rd = EX_MEM_Rd;
    wire [31:0] MEM_ALU_out = EX_MEM_ALU_out;
    wire [31:0] MEM_Mem_out = memOut;

    // MEM/WB pipeline register
    wire [7:0] MEM_WB_Ctrl;
    wire [31:0] MEM_WB_Mem_out, MEM_WB_ALU_out;
    wire [4:0] MEM_WB_Rd;
wire MEM_WB_LUI, MEM_WB_AUIPC;



wire [31:0] MEM_WB_PCplus4;

wire MEM_WB_JALR,MEM_WB_JAL;

N_bit_regsiter #(113) MEM_WB ( // include JALR
    .clk(sclk), .rst(rst), .load(1'b1),
    .data_in({MEM_Ctrl, MEM_Mem_out, MEM_ALU_out, MEM_Rd, EX_MEM_LUI, EX_MEM_AUIPC, EX_MEM_JAL, EX_MEM_JALR, EX_MEM_PCplus4}),
    .data_out({MEM_WB_Ctrl, MEM_WB_Mem_out, MEM_WB_ALU_out, MEM_WB_Rd, MEM_WB_LUI, MEM_WB_AUIPC, MEM_WB_JAL, MEM_WB_JALR, MEM_WB_PCplus4})
);

assign regWriteData = (MEM_WB_JAL || MEM_WB_JALR) ? MEM_WB_PCplus4 : // write return address
                      (MEM_WB_LUI)   ? MEM_WB_ALU_out :                // LUI
                      (MEM_WB_AUIPC) ? MEM_WB_ALU_out :                // AUIPC
                      (MEM_WB_Ctrl[6]) ? MEM_WB_Mem_out :             // MemtoReg
                      MEM_WB_ALU_out;                                  // ALU result

            assign nextPC = (JAL)   ? IF_ID_PC  + immGenOut :
                (EX_MEM_JALR)  ? EX_MEM_ALU_out :
                (EX_MEM_BranchTaken ? EX_MEM_BranchAddOut : PCplus4);
     
          
//always @(posedge sclk or posedge rst) begin
//    if (rst) begin
//        PC_reg <= 32'b0;
//    end else begin
//        if (PCWrite)
//            PC_reg <= nextPC;
//        else
//            PC_reg <= PC_reg; // hold (explicit for clarity)
//    end
//end


    // LED / SSD Display
    always @(*) begin
        case (ledSel)
            2'b00: leds = IF_ID_Inst[15:0];
            2'b01: leds = IF_ID_Inst[31:16];
            2'b10: leds = {ID_EX_Ctrl, EX_MEM_Ctrl[3:0]};
            default: leds = 16'b0;
        endcase
    end

    always @(*) begin
        case (ssdSel)
            4'b0000: ssd = PC[12:0];
            4'b0001: ssd = PCplus4[12:0];
            4'b0010: ssd = nextPC[12:0];
            4'b0011: ssd = EX_MEM_ALU_out[12:0];
            4'b0100: ssd = MEM_WB_ALU_out[12:0];
            default: ssd = 13'b0;
        endcase
    end

endmodule
