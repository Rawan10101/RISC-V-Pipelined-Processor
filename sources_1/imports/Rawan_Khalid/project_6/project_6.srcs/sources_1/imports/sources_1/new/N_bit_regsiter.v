`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/30/2025 04:31:44 PM
// Design Name: 
// Module Name: EXP11
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
module N_bit_regsiter #(
    parameter N = 8
) (
    input clk,
    input rst,
    input load,
    input [N-1:0] data_in,
    output [N-1:0] data_out
);

    wire [N-1:0] mux_out;

    genvar i;
    generate
        for (i = 0; i < N; i = i + 1) begin : loop
            mux2x1 mux (
                .a(data_out[i]),
                .b(data_in[i]),
                .sel(load),
                .y(mux_out[i])
            );

            dff dff1 (
                .clk(clk),
                .reset(rst),
                .d(mux_out[i]),
                .q(data_out[i])
            );
        end
    endgenerate

endmodule

