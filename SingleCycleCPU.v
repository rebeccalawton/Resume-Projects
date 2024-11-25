//Rebecca Lawton and Enya Brett

`define WORD_WIDTH 32
`define NUM_REGS 32
`define OPCODE_COMPUTE    7'b0110011
`define OPCODE_COMPUTE_I  7'b0010011
`define OPCODE_BRANCH     7'b1100011
`define OPCODE_LOAD       7'b0000011
`define OPCODE_STORE      7'b0100011 
`define OPCODE_JAL        7'b1101111
`define OPCODE_JALR       7'b1100111
`define OPCODE_LUI        7'b0110111
`define OPCODE_AUIPC      7'b0010111
`define FUNC_ADD      3'b000
`define FUNC_XOR      3'b100
`define FUNC_OR       3'b110
`define FUNC_AND      3'b111
`define FUNC_SLL      3'b001
`define FUNC_SR       3'b101
`define FUNC_SLT      3'b010
`define FUNC_SLTU     3'b011
`define FUNC_LB       3'b000
`define FUNC_LH       3'b001
`define FUNC_LW       3'b010
`define FUNC_LBU      3'b100
`define FUNC_LHU      3'b101
`define FUNC_BEQ      3'b000
`define FUNC_BNE      3'b001
`define FUNC_BLT      3'b100
`define FUNC_BGE      3'b101
`define FUNC_BLTU     3'b110
`define FUNC_BGEU     3'b111
`define AUX_FUNC_ADD  7'b0000000
`define AUX_FUNC_SUB  7'b0100000
`define AUX_FUNC_ZEROS 7'b0000000
`define AUX_FUNC_SRA  7'b0100000
`define AUX_FUNC_MDR  7'b0000001
`define SIZE_BYTE  2'b00
`define SIZE_HWORD 2'b01
`define SIZE_WORD  2'b10

module SingleCycleCPU(halt, clk, rst);
   output halt;
   input clk, rst;

   wire [`WORD_WIDTH-1:0] PC, InstWord;
   wire [`WORD_WIDTH-1:0] DataAddr, StoreData, DataWord;
   wire [1:0]  MemSize;
   wire        MemWrEn;
   
   wire [4:0]  Rsrc1, Rsrc2, Rdst;
   wire [`WORD_WIDTH-1:0] Rdata1, Rdata2, RWrdata, EU_out, opBin, DataWordSized;
   wire unsigned [`WORD_WIDTH-1:0] Rdata1_unsigned, Rdata2_unsigned;
   wire signed [`WORD_WIDTH-1:0] Rdata1_signed, Rdata2_signed;

   wire        RWrEn;

   wire [`WORD_WIDTH-1:0] NPC, PC_Plus_4;
   wire [6:0]  opcode;

   wire [6:0]  funct7;
   wire [2:0]  funct3;

   wire [31:0] imm_i, imm_b, imm_s, imm_u, imm_j, target;
   wire branch_case;

   wire invalid_op;
   wire valid_load, valid_store, valid_branch, valid_compute, valid_compute_i, valid_extra;
   
   // Only support R-TYPE ADD and SUB
assign halt = invalid_op;
assign valid_load = ((opcode == `OPCODE_LOAD) &&
                              ((funct3 == `FUNC_LB) ||
                              (funct3 == `FUNC_LBU) ||
                              (((funct3 == `FUNC_LH) || (funct3 == `FUNC_LHU)) 
			       && (DataAddr[0] == 1'b0)) ||
                              ((funct3 == `FUNC_LW)
                                       && (DataAddr[1:0] == 2'b00))));
assign valid_store = ((opcode == `OPCODE_STORE) &&
                              ((funct3 == `FUNC_LB) ||
                              ((funct3 == `FUNC_LH)
                                       && (DataAddr[0] == 1'b0)) ||
                              ((funct3 == `FUNC_LW)
                                       && (DataAddr[1:0] == 2'b00))));
assign valid_compute = ((opcode == `OPCODE_COMPUTE) && 
                              (((funct3 == `FUNC_ADD) &&
		                              ((funct7 == `AUX_FUNC_ADD) || (funct7 == `AUX_FUNC_SUB))) ||
                              ((funct3 == `FUNC_XOR) && (funct7 == `AUX_FUNC_ZEROS)) ||
                              ((funct3 == `FUNC_OR) && (funct7 == `AUX_FUNC_ZEROS)) ||
                              ((funct3 == `FUNC_AND) && (funct7 == `AUX_FUNC_ZEROS)) ||
                              ((funct3 == `FUNC_SLL) && (funct7 == `AUX_FUNC_ZEROS)) ||
                              ((funct3 == `FUNC_SR) && 
			       ((funct7 == `AUX_FUNC_ZEROS) || (funct7 == `AUX_FUNC_SRA))) ||
                              ((funct3 == `FUNC_SLT) && (funct7 == `AUX_FUNC_ZEROS)) ||
                              ((funct3 == `FUNC_SLTU) && (funct7 == `AUX_FUNC_ZEROS))));
assign valid_compute_i = ((opcode == `OPCODE_COMPUTE_I) &&
                              ((funct3 == `FUNC_ADD) ||
                              (funct3 == `FUNC_XOR) ||
                              (funct3 == `FUNC_OR) ||
                              (funct3 == `FUNC_AND) ||
                              ((funct3 == `FUNC_SLL) && (funct7 == `AUX_FUNC_ZEROS)) ||
                              ((funct3 == `FUNC_SR) && 
                                    ((funct7 == `AUX_FUNC_ZEROS) || (funct7 == `AUX_FUNC_SRA))) ||
                              (funct3 == `FUNC_SLT) ||
                              (funct3 == `FUNC_SLTU)));
assign valid_branch = ((opcode == `OPCODE_BRANCH) &&
                              ((funct3 == `FUNC_BEQ) ||
                              (funct3 == `FUNC_BNE) ||
                              (funct3 == `FUNC_BLT) ||
                              (funct3 == `FUNC_BGE) ||
                              (funct3 == `FUNC_BLTU) ||
                              (funct3 == `FUNC_BGEU)) &&
                              (imm_b[1:0] == 2'b00));
assign target = Rdata1 + imm_j;
assign valid_extra = (((opcode == `OPCODE_JAL) && (imm_j[1:0] == 2'b00)) ||
                        ((opcode == `OPCODE_JALR) && (funct3 == 3'b000) && (target[1:0] == 2'b00)) ||
                        (opcode == `OPCODE_LUI) ||
                        (opcode == `OPCODE_AUIPC) ||
                        ((opcode == `OPCODE_COMPUTE) && (funct7 == `AUX_FUNC_MDR) &&
			 (funct3 <= 7)));
   assign invalid_op = !( valid_compute ||
                              valid_compute_i ||
                              valid_branch ||
                              valid_load ||
                              valid_store || 
                              valid_extra);

     
   // System State 
   Mem   MEM(.InstAddr(PC), .InstOut(InstWord), 
            .DataAddr(DataAddr), .DataSize(MemSize), .DataIn(StoreData), .DataOut(DataWord), .WE(MemWrEn), .CLK(clk));

   RegFile RF(.AddrA(Rsrc1), .DataOutA(Rdata1), 
	      .AddrB(Rsrc2), .DataOutB(Rdata2), 
	      .AddrW(Rdst), .DataInW(RWrdata), .WenW(RWrEn), .CLK(clk));

   Reg PC_REG(.Din(NPC), .Qout(PC), .WE(1'b1), .CLK(clk), .RST(rst));

   // Instruction Decode
   assign opcode = InstWord[6:0];   
   assign Rdst = InstWord[11:7]; 
   assign Rsrc1 = InstWord[19:15]; 
   assign Rsrc2 = InstWord[24:20];
   assign funct3 = InstWord[14:12];  // R-Type, I-Type, S-Type
   assign funct7 = (opcode == `OPCODE_COMPUTE)? InstWord[31:25] : // R-type
                   ((opcode == `OPCODE_COMPUTE_I) && ((funct3 == `FUNC_SLL) ||
                                                      (funct3 == `FUNC_SR)))? imm_i[11:5] :  // Special I-Type
                   `AUX_FUNC_ZEROS;
      assign imm_i = {{21{InstWord[31]}}, InstWord[30:20]};
      assign imm_s = {{21{InstWord[31]}}, InstWord[30:25], InstWord[11:7]};
      assign imm_b = {{24{InstWord[31]}}, InstWord[7], InstWord[30:25], InstWord[11:8], 1'b0};
	assign imm_u = {InstWord[31:12], {12{1'b0}}}; //for u-type
	assign imm_j = {{12{InstWord[31]}}, InstWord[19:12], InstWord[20], InstWord[30:21], 1'b0}; //for j-type



    // loads and stores
	assign MemWrEn = (opcode == `OPCODE_STORE); // Change this to allow stores
   assign RWrEn = (opcode == `OPCODE_COMPUTE) || // R-type
      (opcode == `OPCODE_COMPUTE_I) || // immediates
      (opcode == `OPCODE_LOAD) || //loads
      ((opcode == `OPCODE_JAL) && (Rdst != 5'b00000)) || // jal
      ((opcode == `OPCODE_JALR) && (Rdst != 5'b00000)) || //jalr
      (opcode == `OPCODE_LUI) || // lui
      (opcode == `OPCODE_AUIPC);  // auipc
   assign DataAddr = (opcode == `OPCODE_LOAD)? Rdata1 + imm_i:
                        Rdata1 + imm_s;
	assign MemSize = ((funct3 == 3'b000) || (funct3 == 3'b100))? `SIZE_BYTE :
                    ((funct3 == 3'b001) || (funct3 == 3'b101))? `SIZE_HWORD :
                    `SIZE_WORD;
   assign StoreData = Rdata2;

   // storage to register file
   assign DataWordSized = (funct3 == 3'b000)? {{24{DataWord[7]}}, DataWord[7:0]} :     //lb
                           (funct3 == 3'b001)? {{16{DataWord[15]}}, DataWord[15:0]} :  //lh
                           (funct3 == 3'b010)? DataWord :                              //lw
                           (funct3 == 3'b100)? {{24{1'b0}}, DataWord[7:0]} :           //lbu
	   			{{16{1'b0}}, DataWord[15:0]};            //lhu

   assign RWrdata = (opcode == `OPCODE_COMPUTE)? EU_out :
	            (opcode == `OPCODE_COMPUTE_I) ? EU_out :
                     (opcode == `OPCODE_LOAD)? DataWordSized :
                     ((opcode == `OPCODE_JAL) || (opcode == `OPCODE_JALR))? PC_Plus_4 :
                     (opcode == `OPCODE_LUI)? imm_u : PC + imm_u;


   // Hardwired to support R-Type instructions -- please add muxes and other control signals
   assign opBin = (opcode == `OPCODE_COMPUTE)? Rdata2 :   //R-type
                  imm_i;                                 //I-type
   ExecutionUnit EU(.out(EU_out), .opA(Rdata1), .opB(opBin), .func(funct3), .auxFunc(funct7));
   assign Rdata1_unsigned = $unsigned(Rdata1);
   assign Rdata2_unsigned = $unsigned(Rdata2);
	assign Rdata1_signed = $signed(Rdata1);
	assign Rdata2_signed = $signed(Rdata2);

   // Fetch Address Datapath
   assign PC_Plus_4 = PC + 4;
	assign branch_case = (opcode == `OPCODE_BRANCH) && (((funct3 == 3'b000) && (Rdata1 == Rdata2)) ||
							    ((funct3 == 3'b001) && (Rdata1 != Rdata2)) ||
							    ((funct3 == 3'b100) && (Rdata1_signed < Rdata2_signed)) ||
							    ((funct3 == 3'b101) && (Rdata1_signed >= Rdata2_signed)) ||
						       ((funct3 == 3'b110) && (Rdata1_unsigned < Rdata2_unsigned)) ||
						       ((funct3 == 3'b111) && (Rdata1_unsigned >= Rdata2_unsigned)));
   assign NPC = (opcode == `OPCODE_JAL)? PC + imm_j :
               (opcode == `OPCODE_JALR)? Rdata1 + imm_i :
	   (branch_case) ? PC + imm_b :
               PC_Plus_4;
   
endmodule // SingleCycleCPU


// Incomplete version of Lab2 execution unit
// You will need to extend it. Feel free to modify the interface also

module ExecutionUnit(
		     output [31:0] out,
		     input [31:0]  opA,
		     input [31:0]  opB,
		     input [2:0]   func,
		     input [6:0]   auxFunc);
   wire [63:0] 			   mulh, mulhsu, mulhu;

   assign mulh = $signed(opA) * $signed(opB);
   assign mulhsu = $signed(opA) * $signed({1'b0, opB});
   assign mulhu = $unsigned(opA) * $unsigned(opB);
   
   //implementation goes here

   //nested use of the ternary operator basically makes 1 enormous mux

  assign out = (func == 3'b000 && auxFunc == 7'b0000000) ? opA + opB :
             (func == 3'b000 && auxFunc == 7'b0100000) ? opA - opB :
             (func == 3'b001 && auxFunc == 7'h0) ? opA << opB[4:0] :
             (func == 3'b010 && auxFunc == 7'h0 && $signed(opA) < $signed(opB)) ? 1 :
             (func == 3'b010 && auxFunc == 7'h0) ? 0 :
             (func == 3'b011 && auxFunc == 7'h0 && $unsigned(opA) < $unsigned(opB)) ? 1 :
             (func == 3'b011 && auxFunc == 7'h0) ? 0 :
             (func == 3'b100 && auxFunc == 7'h0) ? opA ^ opB :
             (func == 3'b101 && auxFunc == 7'b0000000) ? $unsigned(opA) >> opB[4:0] :
             (func == 3'b101 && auxFunc == 7'b0100000) ? $signed( $signed(opA) >>> opB[4:0]) :
             (func == 3'b110 && auxFunc == 7'h0) ? opA | opB :
             (func == 3'b111 && auxFunc == 7'h0) ? opA & opB :
             (func == 3'h0 && auxFunc == 7'h1) ? (opA * opB) & 32'hFFFFFFFF : // Mask lower 32 bits
               (func == 3'h1 && auxFunc == 7'h1) ? mulh[63:32] :
	        (func == 3'h2 && auxFunc == 7'h1) ? mulhsu[63: 32] :
             (func == 3'h3 && auxFunc == 7'h1) ? mulhu[63:32] :

             (func == 3'h4 && auxFunc == 7'h1) ? $signed($signed(opA) / $signed(opB)) :
             (func == 3'h5 && auxFunc == 7'h1) ? $unsigned(opA) / $unsigned(opB) :
             (func == 3'h6 && auxFunc == 7'h1) ? $signed( $signed(opA) % $signed(opB)) :
             (func == 3'h7 && auxFunc == 7'h1) ? $unsigned(opA) % $unsigned(opB) :
             32'b0;
	       
	       // Default case

   endmodule //ExecutionUnit
