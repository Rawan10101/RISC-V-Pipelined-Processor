`timescale 1ns / 1ps

/*******************************************************************
*
* Module: top_riscv_ssd.v
* Project: RISC V Single Cycle Datapath
* Author1: Rawan Muhammad
* Author2: Salma El-Hawary 
* Author1-Email: Rawan_Khalid@aucegypt.edu
* Author2-Email: salma_el-hawary@aucegypt.edu
**********************************************************************/
module N_bit2to1_MUX #(
    parameter N = 8   
)(
    input  [N-1:0] a,   
    input  [N-1:0] b,   
    input sel,          
    output [N-1:0] y    
);

    genvar i;
    generate
        for (i = 0; i < N; i = i + 1) begin : loop
            mux2x1 mux (
                .a(a[i]),
                .b(b[i]),
                .sel(sel),
                .y(y[i])
            );
        end
    endgenerate

endmodule 
