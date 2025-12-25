`timescale 1ns / 1ps
/*******************************************************************
*
* Project: RISC V Single Cycle Datapath
* Author1: Rawan Muhammad
* Author2: Salma El-Hawary 
* Author1-Email: Rawan_Khalid@aucegypt.edu
* Author2-Email: salma_el-hawary@aucegypt.edu
**********************************************************************/
module register_file(
    input [4:0] readreg1,       // rs1
    input [4:0] readreg2,       // rs2
    input [4:0] writereg,       // rd
    input [31:0] writedata,     // data to write to rd
    input clk,
    input reset,
    input regWrite,             // write enable
    output [31:0] readdata1,    // data from rs1
    output [31:0] readdata2     // data from rs2
);

    reg [31:0] regFile [31:0];
    integer i;

 
    always @(negedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 32; i = i + 1)
                regFile[i] <= 32'b0;  // initialize all registers to 0
        end else if (regWrite) begin
            if (writereg != 5'b00000) // x0 is always 0
                regFile[writereg] <= writedata;
        end
    end


    assign readdata1 = (readreg1 == 5'b00000) ? 32'b0 : regFile[readreg1];
    assign readdata2 = (readreg2 == 5'b00000) ? 32'b0 : regFile[readreg2];

endmodule

