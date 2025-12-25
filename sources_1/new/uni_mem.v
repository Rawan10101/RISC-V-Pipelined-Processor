module unified_mem (
    input  wire        sclk,
    input  wire        rst,
    input  wire [7:0]  addr,       // byte address
    input  wire [31:0] write_data, 
    input  wire        mem_write,  
    input  wire        MemRead,
    input  wire [2:0]  inst,       // funct3
//    input  wire        is_unsigned, 
    output reg [31:0]  read_data
);

    reg [7:0] mem [0:255];
    integer i;

    // Initialize memory to 0
    initial begin
        for (i = 0; i < 256; i = i+1)
            mem[i] = 8'b0;
    end

    // Example instructions/data
    initial begin
    mem[0] = 8'b00010011; // bits [7:0]
    mem[1] = 8'b00000000; // bits [15:8]
    mem[2] = 8'b00000000; // bits [23:16]
    mem[3] = 8'b00000000; // bits [31:24]
 
   // ADDI x31,x0,128
    mem[4] = 8'b10010011;  // bits [7:0]
    mem[5] = 8'b0_000_1111;    // bits [15:8]
    mem[6] = 8'b00000000; // bits [23:16]
    mem[7] = 8'b00001000; // bits [31:24]
     
    //mem[1]  = 32'b111111111011_00000_000_00001_0010011; // ADDI x1,x0,-5 -> x1 = -5                                            mem[8]  = 8'b10010011; // 0x93
    mem[9]  = 8'b00000000; // 0x00
    mem[10] = 8'b10110000; // 0xB0
    mem[11] = 8'b11111111; // 0xFF
   
    //mem[2]  = 32'b000000000001_00000_000_00010_0010011; // ADDI x2,x0,1 -> x2 = 1
    mem[12] = 8'b00010011; // 0x13
    mem[13] = 8'b00000001; // 0x01
    mem[14] = 8'b00010000; // 0x10
    mem[15] = 8'b00000000; // 0x00
   
    //mem[3]  = 32'b000000001001_00000_000_00011_0010011; // ADDI x3,x0,9 -> x3 = 9
    mem[16] = 8'b10010011; // 0x93
    mem[17] = 8'b00000001; // 0x03
    mem[18] = 8'b10010000; // 0x90
    mem[19] = 8'b00000000; // 0x00
   
    //mem[4]  = 32'b000000000110_00000_000_00100_0010011; // ADDI x4,x0,6 -> x4 = 6
    mem[20] = 8'b00010011; // 0x13
    mem[21] = 8'b00000010; // 0x02
    mem[22] = 8'b01100000; // 0x60
    mem[23] = 8'b00000000; // 0x00
   
    //mem[4]  = 32'b00001000 0000_0000 0_000_0010 1_0010011; // ADDI x5,x0,128 -> x5 = 128
    mem[24] = 8'b10010011; // 0x13
    mem[25] = 8'b00000010; // 0x05
    mem[26] = 8'b00000000; // 0x80
    mem[27] = 8'b00001000; // 0x00
   
    //mem[5]  = 32'b10000000000000000000_0011 0_0110111; // LUI x6,0x80000  -> x6 = 524288 (decimal) upper 20 to be 2^31      *lui done*
    mem[28] = 8'b00110111; // 0xB7
    mem[29] = 8'b00000011; // 0x06
    mem[30] = 8'b00000000; // 0x00
    mem[31] = 8'b10000000; // 0x80
   
    //mem[6]  = 32'b000000000000_00110_000_0011 1_0010011; // ADDI x7,x6,0 -> x7 = x6 = 2^31
    mem[32] = 8'b10010011; // 0x13
    mem[33] = 8'b00000011; // 0x07
    mem[34] = 8'b00000011; // 0x00
    mem[35] = 8'b00000000; // 0x00
   
    // ADDI x31,x0,128
    mem[36] = 8'b10010011;  // bits [7:0]
    mem[37] = 8'b0_000_1111;    // bits [15:8]
    mem[38] = 8'b00000000; // bits [23:16]
    mem[39] = 8'b00001000; // bits [31:24]
   
    //mem[9] = 32'b0000000_00011_0000 1_111_0001 1_0110011; // AND x3,x1,x3 -> x3 = 9                                          *AND done*
    mem[44] = 8'b10110011; // 0x33
    mem[45] = 8'b11110001; // 0x18
    mem[46] = 8'b00110000; // 0x18
    mem[47] = 8'b00000000; // 0x00
/////////////////////////////////
/////////////////////////////
 //128, 132, 136
//        // ADDI x0,x0,0
//        mem[0] = 8'b00010011; // bits [7:0]
//        mem[1] = 8'b00000000; // bits [15:8]
//        mem[2] = 8'b00000000; // bits [23:16]
//        mem[3] = 8'b00000000; // bits [31:24]
        
//        // ADDI x31,x0,128
//        mem[4] = 8'b10010011;  // bits [7:0]
//        mem[5] = 8'b0_000_1111;    // bits [15:8]
//        mem[6] = 8'b00000000; // bits [23:16]
//        mem[7] = 8'b00001000; // bits [31:24]
        
//        // lw x1, 0(x31) x1=17
//        mem[8]  = 8'b10000011;  // bits [7:0]
//        mem[9]  = 8'b1_010_0000; // bits [15:8]
//        mem[10] = 8'b0000_1111; // bits [23:16]
//        mem[11] = 8'b00000000; // bits [31:24]
        
//        // lw x2, 4(x31) x2=9
//        mem[12] = 8'b00000011;
//        mem[13] = 8'b1_010_0001;
//        mem[14] = 8'b0100_1111;
//        mem[15] = 8'b00000000;
//            // ADDI x2,x0,-10
//        mem[16] = 8'b0_0010011;  // bits [7:0]
//        mem[17] = 8'b0_000_0010;    // bits [15:8]
//        mem[18] = 8'b0110_0000; // bits [23:16]
//        mem[19] = 8'b1111_1111; // bits [31:24]
//        // bgeu x1, x2, 16  taken   x2 =-10 unsigned = 4294967286, x1=17
//        mem[24] = 8'b0_1100011;
//        mem[25] = 8'b1_111_1000;
//        mem[26] = 8'b0010_0000;
//        mem[27] = 8'b00000000;
        
//        // add x3, x1, x2 x3=26
//        mem[28] = 8'b1_0110011;
//        mem[29] = 8'b1_000_0001;
//        mem[30] = 8'b0010_0000;
//        mem[31] = 8'b00000000;
        
//        // add x5, x3, x2 x5=26+9=35
//        mem[32] = 8'b1_0110011;
//        mem[33] = 8'b1_000_0010;
//        mem[34] = 8'b0010_0001;
//        mem[35] = 8'b00000000;
        
//        // add x9, x0, x1 skipped 17
//        mem[36] = 8'b1_0110011;
//        mem[37] = 8'b0_000_0100;
//        mem[38] = 8'b0001_0000;
//        mem[39] = 8'b00000000;
//          // add x9, x9, x1   17+17=34 
//        mem[40] = 8'b1_0110011;
//        mem[41] = 8'b1_000_0100;
//        mem[42] = 8'b0001_0100;
//        mem[43] = 8'b00000000;
   end
    
        //address 128->255 data memory
        initial begin
    
            mem[128] = 8'd17; //i
            mem[132] = 8'd9; 
            mem[136] = 8'd25;  // next word (addr 8) x
        end
    // Synchronous write
    always @(posedge sclk) begin
        if (mem_write) begin
            case (inst)
                3'b000: mem[addr]       <= write_data[7:0];           // SB
                3'b001: begin                                         // SH
                    mem[addr]   <= write_data[7:0];
                    mem[addr+1] <= write_data[15:8];
                end
                3'b010: begin                                         // SW
                    mem[addr]   <= write_data[7:0];
                    mem[addr+1] <= write_data[15:8];
                    mem[addr+2] <= write_data[23:16];
                    mem[addr+3] <= write_data[31:24];
                end
            endcase
        end
    end

    // Combinational read (asynchronous)
    always @(*) begin
        if (mem_write) begin
            read_data = 32'b0;  // writing, ignore output
        end
        else if (MemRead) begin
            // Data memory read
            case (inst)
               3'b000:   read_data = { {24{mem[addr][7]}}, mem[addr]}; //lb 
        3'b001:   read_data = { {16{mem[addr+1][7]}},mem[addr+1], mem[addr]}; //lh
        3'b010:   read_data = {mem[addr+3],mem[addr+2] ,mem[addr+1], mem[addr]}; //lw
        3'b100:   read_data = { {24'b0}, mem[addr]}; //lbu
        3'b101:   read_data = { {16'b0},mem[addr+1], mem[addr]}; //lhu 
                default: read_data = 32'b0;
            endcase
        end
        else begin
            // Instruction fetch (default)
            read_data = {mem[addr+3], mem[addr+2], mem[addr+1], mem[addr]}; // fetch full 32-bit instruction
        end
    end

endmodule


