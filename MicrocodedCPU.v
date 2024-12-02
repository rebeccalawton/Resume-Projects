/*
    Mini LC3 Microcoded Control

    Supports:
     - Operate (ADD,AND,NOT)
     - Memory Transfer (LD/ST)
     - Control (JMP)

This was extended from a CPU designed by Professor Russ Joseph at Northwestern University to include additional microcoded instructions
*/

`define WORD_WIDTH  16
`define REG_COUNT 8
`define OP_SIZE 2
`define RF_ADDR_WIDTH 3
`define MEM_SIZE 65536
`define FN_SIZE 2

`define ADD_OPCODE 4'b0001
`define AND_OPCODE 4'b0101
`define LD_OPCODE 4'b0010
`define ST_OPCODE 4'b0011
`define NOT_OPCODE 4'b1001
`define JMP_OPCODE 4'b1100
`define BR_OPCODE 4'b0000
`define LDR_OPCODE 4'b0110
`define STR_OPCODE 4'b0111

`define UOP_CONTROL_WIDTH 17
`define OPCODE_WIDTH 4
`define STATE_WIDTH  5

module LC3DataPath(
        output [15:0] addr,
        output [15:0] dataWrite,
        input [15:0] dataRead,
        output   dataWrEn,
        input clk, input rst);

    parameter StartAddr = 16'h3000;

   
    wire [`WORD_WIDTH-1:0] SR1_out, SR2_out;
    wire signed [`WORD_WIDTH-1:0] DR_in;
    wire [`RF_ADDR_WIDTH-1:0] SR1, SR2, DR;
    wire [`WORD_WIDTH-1:0] IR;
    wire [`WORD_WIDTH-1:0] alu_out, alu_opa, alu_opb;
    wire [`WORD_WIDTH-1:0] PC, NPC, PC_plus_one;
    wire [`WORD_WIDTH-1:0] immediate, offset, register_offset;
    wire [1:0] alu_fn;

    wire [3:0] opcode;
    wire xtraDispatchInfo;

    wire [4:0] state, next_state;
    wire IR_we, PC_we, CC_we;
    wire [2:0] CC, NCC;

    wire  addr_sel, DR_sel;

      wire   [`UOP_CONTROL_WIDTH-1:0] control_word;
      wire   [`STATE_WIDTH-1:0] dispatch_state;
   wire [1:0] 			state_sel, NPC_sel;

   wire 			alu_opa_sel;
   wire 			[2:0] alu_opb_sel;
   wire branch_taken;

    assign invalid_op = (IR != `ADD_OPCODE) &&
			(IR != `AND_OPCODE) && (IR != `NOT_OPCODE) &&
			(IR != `LD_OPCODE) && (IR != `ST_OPCODE) &&
			(IR != `JMP_OPCODE) && (IR != `BR_OPCODE) && (IR != `LDR_OPCODE) && (IR != `STR_OPCODE);
   
   
      /*  state_sel:
            00 => state + 1
            01 => from dispatch rom
            otherwise => restart (fetch)
       */
   assign next_state = (state_sel == 2'b00) ?  state + 1 :
		   (state_sel == 2'b01) ?  dispatch_state :
		   5'b00000;


    // Basic decode fields
    assign opcode = IR[15:12];
    assign SR1 = (opcode == `STR_OPCODE || opcode == `ST_OPCODE) ? IR[11:9] : IR[8:6];
    assign SR2 = (opcode == `STR_OPCODE) ? IR[8:6] : IR[2:0];
    assign DR = IR[11:9];


    // Various datapath muxes
    assign addr = (addr_sel) ? PC : alu_out;
    assign dataWrite = SR1_out;
    assign alu_opa =  (alu_opa_sel) ? PC_plus_one : (opcode == `STR_OPCODE) ? SR2_out : SR1_out;
    assign alu_opb = (alu_opb_sel == 3'b011 && ( (opcode != `BR_OPCODE) || (branch_taken))) ? offset :
		     (alu_opb_sel == 3'b001) ? immediate : (alu_opb_sel == 3'b010) ? register_offset :
		     (alu_opb_sel == 3'b000) ? SR2_out : 0;
   assign DR_in = (DR_sel) ? dataRead : alu_out;

    // sign extend immediate and offset
    assign immediate = {{12{IR[4]}}, IR[3:0]};
    assign offset = {{8{IR[8]}}, IR[7:0]};
    assign register_offset = {{11{IR[5]}}, IR[4:0]};

    // Next PC Logic
    assign PC_plus_one = PC + 1;
    assign NPC = (NPC_sel == 2'b01) ? SR1_out : (NPC_sel == 2'b11) ? alu_out : PC_plus_one;


   assign {IR_we, PC_we, DR_we, dataWrEn, alu_fn, 
	   addr_sel, alu_opa_sel, alu_opb_sel, DR_sel,
	   NPC_sel, state_sel, CC_we}   = control_word;

     assign NCC[2] = ( $signed(DR_in) < 0); //n bit
     assign NCC[1] = ( $signed(DR_in) == 0); //z bit
     assign NCC[0] = ( $signed(DR_in) > 0); //p bit

     assign xtraDispatchInfo = IR[5];

     assign branch_taken = (CC & IR[11:9]) != 0;

   // Dispatch ROM
   DispatchROM  DPROM(.state(dispatch_state),
		      .opcode(opcode),
		      .xtraDispatchInfo(xtraDispatchInfo));

   // Primary Microcode ROM
   UopROM UROM( .control_word(control_word),
		.state(state));
   
   
    RegFile RF( .SrcAddr1(SR1), .SrcData1(SR1_out), .SrcAddr2(SR2), .SrcData2(SR2_out), 
                .DstAddr(DR), .DstData(DR_in),
                .WriteEn(DR_we), .clk(clk));

    ALU LC3ALU( .result(alu_out),
                .opA(alu_opa),
                .opB(alu_opb),
                .fn(alu_fn));

    // The PC
    Register #(16,StartAddr)  PC_REG  ( .q(PC),     
                  .d(NPC), 
                 .we(PC_we),
                 .clk(clk),
                 .rst(rst));

    // The Instruction Register
    Register #(16)  IR_REG (.q(IR),
                .d(dataRead),
                .we(IR_we),
                .clk(clk),
                .rst(rst));

    Register #(5) CONTRL_REG (.q(state),
                .d(next_state),
                .we(1'b1),
                .clk(clk),
                .rst(rst));

    Register #(3) NZP_REG (.q(CC), 
    .d(NCC), 
    .we(CC_we),
    .clk(clk), 
    .rst(rst));

endmodule // LC3DataPath

module UopROM(
	      output [`UOP_CONTROL_WIDTH-1:0] control_word,
	      input [`STATE_WIDTH-1:0] 	      state);
   reg [`UOP_CONTROL_WIDTH-1:0] 	      control_word;
   

    /* Control Word = {
           IR_we, PC_we, DR_we, dataWrEn, 
           alu_fn[1:0], 
           addr_sel, alu_opa_sel, alu_opb_sel[2:0], DR_sel,
	   NPC_sel[1:0], state_sel[1:0], CC_we }
     */

   
   always @* begin
     case (state)
       5'b00000: control_word = 17'b10000010000000000; // FETCH
       5'b00001: control_word = 17'b00000000000000010; // DECODE
       5'b00010: control_word = 17'b01000000000001110; // JMP
       5'b00011: control_word = 17'b01101100000000111; // NOT
       5'b00100: control_word = 17'b01100000001000111; // ADDI
       5'b00101: control_word = 17'b01100000000000111; // ADDR
       5'b00110: control_word = 17'b01101000001000111; // ANDI
       5'b00111: control_word = 17'b01101000000000111; // ANDR
       5'b01000: control_word = 17'b00000001011000000; // LD 0
       5'b01001: control_word = 17'b01100001011100111; // LD 1
       5'b01100: control_word = 17'b00000001011000000; // ST 0
       5'b01101: control_word = 17'b01010001011000110; // ST 1
       5'b01110: control_word = 17'b00000001011011000; // BR 0
       5'b01111: control_word = 17'b01000001011011110; // BR 1
       5'b10000: control_word = 17'b00000000010000000; // LDR 0
       5'b10001: control_word = 17'b01100000010100111; // LDR 1
       5'b10010: control_word = 17'b00000000010000000; // STR 0
       5'b10011: control_word = 17'b01010000010000110; // STR 1      

       default: control_word = 0;
     endcase // case (state)
   end // always @ *

endmodule // UopROM

module DispatchROM(
		   output [`STATE_WIDTH-1:0] state,
		   input [`OPCODE_WIDTH-1:0] opcode,
		   input 		     xtraDispatchInfo);
   reg [`STATE_WIDTH-1:0] 		     state;

   
   always @* begin
     casex ({opcode,xtraDispatchInfo})
       5'b00010: state = 5'b00101; // ADDR
       5'b00011: state = 5'b00100; // ADDI
       5'b01010: state = 5'b00111; // ANDR
       5'b01011: state = 5'b00110; // ANDI
       5'b1001x: state = 5'b00011; // NOT
       5'b1100x: state = 5'b00010; // JMP
       5'b0010x: state = 5'b01000; // LD
       5'b0011x: state = 5'b01100; // ST
       5'b0000x: state = 5'b01110; //BR
       5'b0110x: state = 5'b10000; //LDR
       5'b0111x: state = 5'b10010; //STR
       default: state = 0;
     endcase // casex ({opcode,xtraDispatchInfo})
   end // always @ *

endmodule // DispatchROM


module LC3Memory(
    input [`WORD_WIDTH-1:0] addr, 
    input [`WORD_WIDTH-1:0] DataWrite, 
    output [`WORD_WIDTH-1:0] DataRead,
    input WriteEn, 
    input clk);

    reg [`WORD_WIDTH-1:0] mem[0:`MEM_SIZE-1];

    assign DataRead = mem[addr];

    always @ (posedge clk) begin
        if (WriteEn)
            mem[addr] <= DataWrite;
    end


endmodule

module RegFile(
    input [`RF_ADDR_WIDTH-1:0] SrcAddr1, 
    output [`WORD_WIDTH-1:0] SrcData1, 
    input [`RF_ADDR_WIDTH-1:0] SrcAddr2, 
    output [`WORD_WIDTH-1:0] SrcData2, 
    input [`RF_ADDR_WIDTH-1:0] DstAddr,
    input [`WORD_WIDTH-1:0] DstData,
    input WriteEn, 
    input clk);

    reg [`WORD_WIDTH-1:0] mem[0:`REG_COUNT-1];

    assign SrcData1 = mem[SrcAddr1];
    assign SrcData2 = mem[SrcAddr2];

    always @ (posedge clk)
        if (WriteEn)
            mem[DstAddr] <= DstData;
endmodule

module ALU(
    output [`WORD_WIDTH-1:0] result,
    input [`WORD_WIDTH-1:0] opA,
    input [`WORD_WIDTH-1:0] opB,
    input [`FN_SIZE-1:0] fn);

    wire [`WORD_WIDTH-1:0] addResult, andResult, notResult;

    assign addResult = opA + opB;
    assign andResult = opA & opB;
    assign notResult = ~opA;

    assign result = (fn == 2'b11) ? notResult : 
        (fn[1] == 1'b0) ? addResult : andResult;

endmodule

module Register(
    output [width-1:0] q, 
    input [width-1:0] d,
    input we,
    input clk,
    input rst);
    parameter width = 16;
    parameter rst_value = 0;

    reg [width-1:0] q;

    always @ (posedge clk or negedge rst)
        if (~rst)
            q <= rst_value;
        else 
            q <= (we) ? d : q;

endmodule
