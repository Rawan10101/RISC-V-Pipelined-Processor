`timescale 1ns / 1ps

/*******************************************************************
*
* Project: RISC V Single Cycle Datapath
* Author1: Rawan Muhammad
* Author2: Salma El-Hawary 
* Author1-Email: Rawan_Khalid@aucegypt.edu
* Author2-Email: salma_el-hawary@aucegypt.edu
**********************************************************************/

module RISC_V_tb();

    reg clk;
    reg rst;
    reg [1:0] ledSel;
    reg [4:0] ssdSel;
    wire [15:0] leds;
    wire [12:0] ssd;

    integer i;

    RISC_V uut (
        .clk(clk),
        .rst(rst),
        .ledSel(ledSel),
        .ssdSel(ssdSel),
        .leds(leds),
        .ssd(ssd)
    );

    always #5 clk = ~clk;

    initial begin
        clk = 0; rst = 1; ledSel = 0; ssdSel = 0;

        #5 rst = 0;

        $display("\n--- Cycle 0 ---");

        // LED display
        ledSel = 0;
        for (ledSel = 0; ledSel < 3; ledSel = ledSel + 1) begin
 #5             $display("ledSel=%b  leds=%h", ledSel, leds);
        end

        // SSD display
        ssdSel = 0;
        for (ssdSel = 0; ssdSel < 12; ssdSel = ssdSel + 1) begin
 #5             $display("ssdSel=%02b  ssd=%h", ssdSel, ssd);
        end

        // Now cycle instructions properly
        for (i = 0; i < 11; i = i + 1) begin
            @(posedge clk);

            $display("\n--- Cycle %0d ---", i+1);

            ledSel = 0;
            for (ledSel = 0; ledSel < 3; ledSel = ledSel + 1) begin
 #5               $display("ledSel=%b  leds=%h", ledSel, leds);
            end

            ssdSel = 0;
            for (ssdSel = 0; ssdSel < 12; ssdSel = ssdSel + 1) begin
 #5               $display("ssdSel=%02b  ssd=%h", ssdSel, ssd);
            end
        end

//        $finish;
    end

endmodule


