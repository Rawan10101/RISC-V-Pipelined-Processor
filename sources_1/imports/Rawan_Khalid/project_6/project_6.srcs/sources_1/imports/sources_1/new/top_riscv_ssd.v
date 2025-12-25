`timescale 1ns / 1ps
/*******************************************************************
* Project: RISC V Single Cycle Datapath
* Author1: Rawan Muhammad
* Author2: Salma El-Hawary 
* Author1-Email: Rawan_Khalid@aucegypt.edu
* Author2-Email: salma_el-hawary@aucegypt.edu
* Description: Integeration of ssd and RISC V module-Top Module
**********************************************************************/

module top_riscv_ssd(
    input wire clk100,          
    input wire rclk,       
    input wire [1:0] sw_ledSel, 
    input wire [3:0] sw_ssdSel, 
    output wire [15:0] LED,     
    output wire [6:0] seg,     
    output wire [3:0] an       
);
    wire [12:0] ssd_val;

    RISC_V FF (
        .clk(rclk),         
        .rst(0),          
        .ledSel(sw_ledSel),    
        .ssdSel(sw_ssdSel),     
        .leds(LED),             
        .ssd(ssd_val)           
    );

    ssd_driver DISP (
        .clk(clk100),           
        .num(ssd_val),        
        .Anode(an),               
        .LED_out(seg)               
    );

endmodule
