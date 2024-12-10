//Rebecca Lawton and Enya Brett
//RISC-V Pipelined CPU design
//implements RV32IM

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

module PipelinedCPU(halt, clk, rst);
   output halt;
   input clk, rst;

	wire [`WORD_WIDTH-1:0] PC, InstWord, InstWord_id, InstWord_de, InstWord_ex, InstWord_mw;
   wire [`WORD_WIDTH-1:0] DataAddr, StoreData, DataWord, DataWord_mw;
   wire [1:0]  MemSize;
   wire        MemWrEn;
   
   wire [4:0]  Rsrc1, Rsrc2, Rdst, Rdst_de, Rdst_em, Rdst_mw;
	wire [`WORD_WIDTH-1:0] Rdata1, Rdata2, RWrdata, alu_out, writeback, opAin, opBin, DataWordSized, first_reg_data, second_reg_data;
   wire [`WORD_WIDTH-1:0] Rdata1_de, Rdata2_de, Rdata2_em, store_after_load_RD2, alu_out_em, alu_out_mw;
   wire unsigned [`WORD_WIDTH-1:0] Rdata1_unsigned, Rdata2_unsigned;
   wire signed [`WORD_WIDTH-1:0] Rdata1_signed, Rdata2_signed;

   wire        RWrEn, writes_to_reg;

   wire [`WORD_WIDTH-1:0] NPC, PC_Plus_4, PC_id, PC_de, PC_em, PC_mw;
   wire [6:0]  opcode, opcode_de, opcode_em, opcode_mw;

   wire [6:0]  funct7, funct7_de, auxfunct_in;
   wire [2:0]  funct3, funct3_de, funct3_em, funct3_mw, funct_in;

   wire [31:0] imm_i, imm_b, imm_s, imm_u, imm_j, jalr_loc;
   wire [31:0] imm_i_de, imm_b_de, imm_s_de, imm_u_de, imm_j_de;
   wire [31:0] imm_u_em, imm_j_em, imm_u_mw;
   wire branch_case, jal_case, jalr_case, flush_case;
   wire checkrs1, checkrs2;
	wire RAW_hazard, first_op_hazard, second_op_hazard, double_hazard, first_op_hazard_id, second_op_hazard_id, first_op_hazard_de, second_op_hazard_de;

   wire invalid_op, halt_in, halt_id, halt_de, halt_em, halt_mw;
   wire valid_load, valid_store, valid_branch, valid_compute, valid_compute_i, valid_extra;
	wire [14:0] recent_RD, nextRDList, recent_dests;
   wire valid_flush, active_dec, active_ex, active_mem, active_wb;
	wire [1:0] first_op_hazard_distance, second_op_hazard_distance,
	first_op_hazard_d_id, second_op_hazard_d_id, first_op_hazard_d_de, second_op_hazard_d_de;

	assign recent_RD = (PC_id == 0) ? 15'b0 : recent_dests;
	assign writes_to_reg = ((InstWord_id[6:0] == `OPCODE_COMPUTE) || // R-type
				(InstWord_id[6:0] == `OPCODE_COMPUTE_I) || // immediates
				(InstWord_id[6:0] == `OPCODE_LOAD) || //loads
				(InstWord_id[6:0] == `OPCODE_JAL) || // jal
				(InstWord_id[6:0] == `OPCODE_JALR) || //jalr
				(InstWord_id[6:0] == `OPCODE_LUI) || // lui
				(InstWord_id[6:0] == `OPCODE_AUIPC) && (Rdst != 5'b00000));
	assign nextRDList = (writes_to_reg) ? {InstWord_id[11:7], recent_RD[14:5]} : {5'b0, recent_RD[14:5]};
	assign checkrs1 = !((InstWord_id[6:0] == `OPCODE_LUI) || (InstWord_id[6:0] == `OPCODE_JAL) || (InstWord_id[6:0] == `OPCODE_AUIPC));
	assign checkrs2 = (InstWord_id[6:0] == `OPCODE_BRANCH) || (InstWord_id[6:0] == `OPCODE_STORE) || (InstWord_id[6:0] == `OPCODE_COMPUTE);
	assign RAW_hazard = (PC_id == 0) ? 0 : (((InstWord_id[19:15] == recent_RD[14:10]) && (recent_RD[14:10] != 0) && checkrs1) || ((InstWord_id[19:15] == recent_RD[9:5]) && (recent_RD[9:5] != 0) && checkrs1) || ((InstWord_id[19:15] == recent_RD[4:0]) && (recent_RD[4:0] != 0) && checkrs1) 
						|| ((InstWord_id[24:20] == recent_RD[14:10]) && (recent_RD[14:10] != 0) && checkrs2) || ((InstWord_id[24:20] == recent_RD[9:5]) && (recent_RD[9:5] != 0) && checkrs2) || ((InstWord_id[24:20] == recent_RD[4:0]) && (recent_RD[4:0] != 0) && checkrs2 ));

	//is the hazard caused by the first, second, or 3rd instruction above the current one?

	//was the hazard caused by the first or second operand of the current instruction?

	assign first_op_hazard = ((InstWord_id[19:15] == recent_RD[14:10]) && (recent_RD[14:10] != 0) && checkrs1) || ((InstWord_id[19:15] == recent_RD[9:5]) && (recent_RD[9:5] != 0) && checkrs1) || ((InstWord_id[19:15] == recent_RD[4:0]) && (recent_RD[4:0] != 0) && checkrs1);
	assign first_op_hazard_distance = (InstWord_id[19:15] == recent_RD[14:10]) ? 1 : (InstWord_id[19:15] == recent_RD[9:5])  ? 2 : 3;
	assign second_op_hazard = ((InstWord_id[24:20] == recent_RD[14:10]) && (recent_RD[14:10] != 0) && checkrs2) || ((InstWord_id[24:20] == recent_RD[9:5]) && (recent_RD[9:5] != 0) && checkrs2) || ((InstWord_id[24:20] == recent_RD[4:0]) && (recent_RD[4:0] != 0) && checkrs2);
	assign second_op_hazard_distance = (InstWord_id[24:20] == recent_RD[14:10]) ? 1 : (InstWord_id[24:20] == recent_RD[9:5]) ? 2 : 3;
	assign double_hazard = first_op_hazard && second_op_hazard;
	
   //Register to keep track of RAW Hazards. Stores destination registers of last 3 instructions
	Register #(15) RD_Reg (.q(recent_dests), 
		       .d(nextRDList), 
			  .we(!mul_hold && !div_hold), 
		       .clk(clk), 
			       .rst(rst));
	
   Register #(1) halt_reg_id (.q(halt_id), .d(halt_in), .we(!mul_hold && !div_hold), .clk(clk), .rst(rst));
   Register #(1) halt_reg_de (.q(halt_de), .d(halt_id || (active_mem && !valid_DataAddr) ||
                        (active_dec && !valid_jalr)), .we(!mul_hold && !div_hold), .clk(clk), .rst(rst));
   Register #(1) halt_reg_em (.q(halt_em), .d(halt_de || (active_mem && !valid_DataAddr)), .we(!mul_hold && !div_hold), .clk(clk), .rst(rst));
   Register #(1) halt_reg_mw (.q(halt_mw), .d(halt_em), .we(!mul_hold && !div_hold), .clk(clk), .rst(rst));
   assign halt_in = invalid_op;
   assign halt = (halt_in && halt_id && halt_de && halt_em && halt_mw);
   assign valid_load = ((opcode == `OPCODE_LOAD) &&
                              ((funct3 == `FUNC_LB) ||
                              (funct3 == `FUNC_LBU) ||
                              (funct3 == `FUNC_LH) || 
                              (funct3 == `FUNC_LHU) ||
                              (funct3 == `FUNC_LW)));
   assign valid_store = ((opcode == `OPCODE_STORE) &&
                              ((funct3 == `FUNC_LB) ||
                               (funct3 == `FUNC_LH) ||
                               (funct3 == `FUNC_LW)));
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
   assign valid_extra = (((opcode == `OPCODE_JAL) && (imm_j[1:0] == 2'b00)) ||
                        ((opcode == `OPCODE_JALR) && (funct3 == 3'b000)) ||
                        (opcode == `OPCODE_LUI) ||
                        (opcode == `OPCODE_AUIPC) ||
                        ((opcode == `OPCODE_COMPUTE) && (funct7 == `AUX_FUNC_MDR) &&
			 (funct3 <= 7)));
   assign invalid_op = (active_dec && !(valid_compute ||
                              valid_compute_i ||
                              valid_branch ||
                              valid_load ||
                              valid_store || 
                              valid_extra)) ||
                        (active_mem && !valid_DataAddr) ||
                        (active_dec && !valid_jalr);


//------------------------------------------------------------------------------------------------------------  
   //MULT/DIV HOLD IMPLEMENTATION

   wire mul_hold, div_hold;
   assign mul_hold = (opcode_de == `OPCODE_COMPUTE) && (funct7_de == 7'b0000001) && (funct3_de < 4) && (mul_count<3);
   assign div_hold = (opcode_de == `OPCODE_COMPUTE) && (funct7_de == 7'b0000001) && (funct3_de >= 4) && (div_count<19);
   wire [7:0] mul_count, mul_next_count, div_count, div_next_count;
   assign mul_next_count = (mul_hold)? mul_count + 1: 0;
   assign div_next_count = (div_hold)? div_count + 1: 0;
   Register #(8) MulCounter (.q(mul_count), 
			       .d(mul_next_count),
			       .we(1'b1),
			       .clk(clk),
			       .rst(rst));

   Register #(8) DivCounter (.q(div_count), 
			       .d(div_next_count),
			       .we(1'b1),
			       .clk(clk),
			       .rst(rst));


//------------------------------------------------------------------------------------------------------------

   // System State 
	Mem   MEM(.InstAddr(PC), .InstOut(InstWord), 
            .DataAddr(DataAddr), .DataSize(MemSize), .DataIn(StoreData), .DataOut(DataWord), .WE(MemWrEn), .CLK(clk));

   RegFile RF(.AddrA(Rsrc1), .DataOutA(Rdata1), 
	      .AddrB(Rsrc2), .DataOutB(Rdata2), 
	      .AddrW(Rdst_mw), .DataInW(RWrdata), .WenW(RWrEn), .CLK(clk));

	Reg PC_REG(.Din(NPC), .Qout(PC), .WE(!halt_in && !mul_hold && !div_hold), .CLK(clk), .RST(rst));


//------------------------------------------------------------------------------------------------------------
   // Instruction fetch stage registers
   assign PC_Plus_4 = PC + 4;
   Register #(32) InstWord_reg_id (.q(InstWord_id), .d(InstWord), .we(!mul_hold && !div_hold), .clk(clk), .rst(rst));
   Register #(32) PC_reg_id (.q(PC_id), .d(PC), .we(!mul_hold && !div_hold), .clk(clk), .rst(rst));
   Register #(1) if_active (.q(active_dec), .d(1'b1), .we(1'b1), .clk(clk), .rst(rst));
	Register #(1) first_hazard_id (.q(first_op_hazard_id), .d(first_op_hazard), .we(!mul_hold && !div_hold), .clk(clk), .rst(rst));
	Register #(1) second_hazard_id (.q(second_op_hazard_id), .d(second_op_hazard), .we(!mul_hold && !div_hold), .clk(clk), .rst(rst));
	Register #(2) first_distance_id (.q(first_op_hazard_d_id), .d(first_op_hazard_distance), .we(!mul_hold && !div_hold), .clk(clk), .rst(rst));
	Register #(2) second_distance_id (.q(second_op_hazard_d_id), .d(second_op_hazard_distance), .we(!mul_hold && !div_hold), .clk(clk), .rst(rst));


//------------------------------------------------------------------------------------------------------------
   // Instruction Decode
   assign opcode = InstWord_id[6:0];   
   assign Rdst = InstWord_id[11:7]; 
   assign Rsrc1 = InstWord_id[19:15]; 
   assign Rsrc2 = InstWord_id[24:20];
   assign funct3 = InstWord_id[14:12];  // R-Type, I-Type, S-Type
   assign funct7 = (opcode == `OPCODE_COMPUTE)? InstWord_id[31:25] : // R-type
                   ((opcode == `OPCODE_COMPUTE_I) && ((funct3 == `FUNC_SLL) ||
                                                      (funct3 == `FUNC_SR)))? imm_i[11:5] :  // Special I-Type
                   `AUX_FUNC_ZEROS;
   assign imm_i = {{21{InstWord_id[31]}}, InstWord_id[30:20]};
   assign imm_s = {{21{InstWord_id[31]}}, InstWord_id[30:25], InstWord_id[11:7]};
   assign imm_b = {{24{InstWord_id[31]}}, InstWord_id[7], InstWord_id[30:25], InstWord_id[11:8], 1'b0};
	assign imm_u = {InstWord_id[31:12], {12{1'b0}}}; //for u-type
	assign imm_j = {{12{InstWord_id[31]}}, InstWord_id[19:12], InstWord_id[20], InstWord_id[30:21], 1'b0}; //for j-type

	assign first_reg_data = (first_op_hazard_id && first_op_hazard_d_id == 3) ? RWrdata : Rdata1;
	assign second_reg_data = (second_op_hazard_id && second_op_hazard_d_id == 3) ? RWrdata :
                           (second_op_hazard && (second_op_hazard_distance == 1) && (opcode == `OPCODE_STORE))? writeback :
                           (second_op_hazard && (second_op_hazard_distance == 2) && (opcode == `OPCODE_STORE))? alu_out_em : Rdata2; 

   assign dec_en = active_dec && !mul_hold && !div_hold && ((opcode != `OPCODE_JALR) || valid_jalr)  && (((opcode_em != `OPCODE_LOAD) && (opcode_em != `OPCODE_STORE)) || valid_DataAddr);

   assign jalr_loc = first_reg_data + imm_j;
   assign valid_jalr = (opcode != `OPCODE_JALR) || (jalr_loc[1:0] == 2'b00);

   // Decode stage registers
   Register #(7) opcode_reg_de (.q(opcode_de), .d(opcode), .we(dec_en), .clk(clk), .rst(rst));
   Register #(5) Rdst_reg_de   (.q(Rdst_de), .d(Rdst), .we(dec_en), .clk(clk), .rst(rst));
	Register #(32) Rdata1_reg_de (.q(Rdata1_de), .d(first_reg_data), .we(dec_en), .clk(clk), .rst(rst));
	Register #(32) Rdata2_reg_de (.q(Rdata2_de), .d(second_reg_data), .we(dec_en), .clk(clk), .rst(rst));
   Register #(3) funct3_reg_de (.q(funct3_de), .d(funct3), .we(dec_en), .clk(clk), .rst(rst));
   Register #(7) funct7_reg_de (.q(funct7_de), .d(funct7), .we(dec_en), .clk(clk), .rst(rst));
   Register #(32) imm_i_reg_de (.q(imm_i_de), .d(imm_i), .we(dec_en), .clk(clk), .rst(rst));
   Register #(32) imm_s_reg_de (.q(imm_s_de), .d(imm_s), .we(dec_en), .clk(clk), .rst(rst));
   Register #(32) imm_b_reg_de (.q(imm_b_de), .d(imm_b), .we(dec_en), .clk(clk), .rst(rst));
   Register #(32) imm_u_reg_de (.q(imm_u_de), .d(imm_u), .we(dec_en), .clk(clk), .rst(rst));
   Register #(32) imm_j_reg_de (.q(imm_j_de), .d(imm_j), .we(dec_en), .clk(clk), .rst(rst));
   Register #(32) PC_reg_de (.q(PC_de), .d(PC_id), .we(dec_en), .clk(clk), .rst(rst));
   Register #(1) dec_active (.q(active_ex), .d(active_dec && valid_flush), .we(1'b1), .clk(clk), .rst(rst));
	Register #(1) first_hazard_de (.q(first_op_hazard_de), .d(first_op_hazard_id), .we(dec_en), .clk(clk), .rst(rst));
	Register #(1) second_hazard_de (.q(second_op_hazard_de), .d(second_op_hazard_id), .we(dec_en), .clk(clk), .rst(rst));
	Register #(2) first_distance_de (.q(first_op_hazard_d_de), .d(first_op_hazard_d_id), .we(dec_en), .clk(clk), .rst(rst));
	Register #(2) second_distance_de (.q(second_op_hazard_d_de), .d(second_op_hazard_d_id), .we(dec_en), .clk(clk), .rst(rst));
	Register #(32) InstWord_reg_de (.q(InstWord_de), .d(InstWord_id), .we(dec_en), .clk(clk), .rst(rst));


   
//------------------------------------------------------------------------------------------------------------
   // Execution stage
	assign opAin = (first_op_hazard_id && first_op_hazard_d_id == 1 && opcode_em != `OPCODE_LOAD) ? alu_out_em : 
		(first_op_hazard_id && first_op_hazard_d_id == 2 && opcode_mw != `OPCODE_LOAD) ? alu_out_mw : 
		(first_op_hazard_id && first_op_hazard_d_id == 1 && opcode_em == `OPCODE_LOAD) ? DataWord :
		(first_op_hazard_id && first_op_hazard_d_id == 2 && opcode_mw == `OPCODE_LOAD) ? DataWord_mw :
		((opcode_de == `OPCODE_AUIPC) || (opcode_de == `OPCODE_LUI))? imm_u_de : Rdata1_de;

	
	assign opBin = (second_op_hazard_id && second_op_hazard_d_id == 1 && (opcode_em != `OPCODE_LOAD) && (opcode_de != `OPCODE_STORE)) ? alu_out_em : 
		(second_op_hazard_id && second_op_hazard_d_id == 2 && (opcode_mw != `OPCODE_LOAD) && (opcode_de != `OPCODE_STORE)) ? alu_out_mw : 
		(second_op_hazard_id && second_op_hazard_d_id == 1 && (opcode_em == `OPCODE_LOAD) && (opcode_de != `OPCODE_STORE)) ? DataWord :
		(second_op_hazard_id && second_op_hazard_d_id == 2 && (opcode_mw == `OPCODE_LOAD) && (opcode_de != `OPCODE_STORE)) ? DataWord_mw :
	   ((opcode_de == `OPCODE_AUIPC) || (opcode_de == `OPCODE_LUI))? 32'b00000000000000000000000000001100 :
                  ((opcode_de == `OPCODE_COMPUTE) || (opcode_de == `OPCODE_BRANCH))? Rdata2_de :   //R-type
                  ((opcode_de == `OPCODE_COMPUTE_I) || (opcode_de == `OPCODE_LOAD))? imm_i_de:   //I-type
                  imm_s_de;
	
   assign funct_in =  ((opcode_de == `OPCODE_AUIPC) || (opcode_de == `OPCODE_LUI))? `FUNC_SLL :
                     ((opcode_de == `OPCODE_BRANCH) && ((funct3_de == `FUNC_BEQ) ||
                                                       (funct3_de == `FUNC_BNE)))? `FUNC_ADD:
                     ((opcode_de == `OPCODE_BRANCH) && ((funct3_de == `FUNC_BLT) ||
                                                       (funct3_de == `FUNC_BGE)))? `FUNC_SLT: 
                     ((opcode_de == `OPCODE_BRANCH) && ((funct3_de == `FUNC_BEQ) ||
                                                       (funct3_de == `FUNC_BNE)))? `FUNC_SLTU:
                     ((opcode_de == `OPCODE_LOAD) || (opcode_de == `OPCODE_STORE))? `FUNC_ADD:
                     funct3_de;
   assign auxfunct_in = ((opcode_de == `OPCODE_AUIPC) || (opcode_de == `OPCODE_LUI))? `AUX_FUNC_ZEROS :
                        ((opcode_de == `OPCODE_BRANCH) && ((funct3_de == `FUNC_BEQ) ||
                                                       (funct3_de == `FUNC_BNE)))? `AUX_FUNC_SUB:
                        (opcode_de == `OPCODE_BRANCH)? `AUX_FUNC_ZEROS:
                        ((opcode_de == `OPCODE_LOAD) || (opcode_de == `OPCODE_STORE))? `AUX_FUNC_ADD:
                        funct7_de;
   
   assign writeback = (opcode_de == `OPCODE_AUIPC)? PC_de + alu_out :
                        ((opcode_de == `OPCODE_JAL) || (opcode_de == `OPCODE_JALR))? PC_de + 4:
                        alu_out;

   ExecutionUnit EU(.out(alu_out), .opA(opAin), .opB(opBin), .func(funct_in), .auxFunc(auxfunct_in));
   
   assign branch_case = (opcode_de == `OPCODE_BRANCH) && 
                        (((funct3_de == `FUNC_BEQ) && (alu_out == 0)) ||
							    ((funct3_de == `FUNC_BNE) && (alu_out != 0)) ||
							    ((funct3_de == `FUNC_BLT) && (alu_out)) ||
							    ((funct3_de == `FUNC_BGE) && (!(alu_out))) ||
						       ((funct3_de == `FUNC_BLTU) && (alu_out)) ||
						       ((funct3_de == `FUNC_BGEU) && (!(alu_out))));
   assign jal_case = (opcode_de == `OPCODE_JAL);
   assign jalr_case = (opcode_de == `OPCODE_JALR);
   assign flush_case = (branch_case || jal_case || jalr_case) && active_ex;
   assign valid_flush = !(flush_case && active_ex);

   assign store_after_load_RD2 = ((opcode_de == `OPCODE_STORE) && (opcode_em == `OPCODE_LOAD) && second_op_hazard_id && (second_op_hazard_d_id == 1))? DataWord: 
                                 ((opcode_de == `OPCODE_STORE) && (opcode_mw == `OPCODE_LOAD) && second_op_hazard_id && (second_op_hazard_d_id == 2))? RWrdata:
                                 Rdata2_de;

   assign ex_en = active_ex && !mul_hold && !div_hold && (((opcode_em != `OPCODE_LOAD) && (opcode_em != `OPCODE_STORE)) || valid_DataAddr);

   Register #(7) opcode_reg_em (.q(opcode_em), .d(opcode_de), .we(ex_en), .clk(clk), .rst(rst));
   Register #(3) funct3_reg_em (.q(funct3_em), .d(funct3_de), .we(ex_en), .clk(clk), .rst(rst));
   Register #(32) alu_out_reg_em (.q(alu_out_em), .d(writeback), .we(ex_en), .clk(clk), .rst(rst));
   Register #(32) imm_u_reg_em (.q(imm_u_em), .d(imm_u_de), .we(ex_en), .clk(clk), .rst(rst));
   Register #(32) imm_j_reg_em (.q(imm_j_em), .d(imm_j_de), .we(ex_en), .clk(clk), .rst(rst));
   Register #(32) Rdata2_reg_em (.q(Rdata2_em), .d(store_after_load_RD2), .we(ex_en), .clk(clk), .rst(rst));
   Register #(5) Rdst_reg_em (.q(Rdst_em), .d(Rdst_de), .we(ex_en), .clk(clk), .rst(rst));
   Register #(32) PC_reg_em (.q(PC_em), .d(PC_de), .we(ex_en), .clk(clk), .rst(rst));
   Register #(1) ex_active (.q(active_mem), .d(active_ex), .we(1'b1), .clk(clk), .rst(rst));
	Register #(32) InstWord_reg_ex (.q(InstWord_ex), .d(InstWord_de), .we(ex_en), .clk(clk), .rst(rst));


//------------------------------------------------------------------------------------------------------------   
   // loads and stores
	assign MemWrEn = (opcode_em == `OPCODE_STORE); // Change this to allow stores
   assign DataAddr = alu_out_em;
	assign MemSize = ((funct3_em == 3'b000) || (funct3_em == 3'b100))? `SIZE_BYTE :
                    ((funct3_em == 3'b001) || (funct3_em == 3'b101))? `SIZE_HWORD :
                    `SIZE_WORD;
   assign StoreData = Rdata2_em;

   assign valid_DataAddr = !((opcode_em == `OPCODE_LOAD) || (opcode_em == `OPCODE_STORE)) ||
                                    ((((funct3_em == `FUNC_LH) || (funct3_em == `FUNC_LHU)) 
			                              && (alu_out_em[0] == 1'b0)) ||
                                    ((funct3_em == `FUNC_LW)
                                       && (alu_out_em[1:0] == 2'b00)));

   assign mem_en = active_mem && !mul_hold && !div_hold && (((opcode_em != `OPCODE_LOAD) && (opcode_em != `OPCODE_STORE)) || valid_DataAddr);     

   Register #(7) opcode_reg_mw (.q(opcode_mw), .d(opcode_em), .we(mem_en), .clk(clk), .rst(rst));
   Register #(3) funct3_reg_mw (.q(funct3_mw), .d(funct3_em), .we(mem_en), .clk(clk), .rst(rst));
   Register #(32) alu_out_reg_mw (.q(alu_out_mw), .d(alu_out_em), .we(mem_en), .clk(clk), .rst(rst));
   Register #(32) DataWord_reg_mw (.q(DataWord_mw), .d(DataWord), .we(mem_en), .clk(clk), .rst(rst));
   Register #(32) imm_u_reg_mw (.q(imm_u_mw), .d(imm_u_em), .we(mem_en), .clk(clk), .rst(rst));
   Register #(32) PC_reg_mw (.q(PC_mw), .d(PC_em), .we(mem_en), .clk(clk), .rst(rst));
   Register #(5) Rdst_reg_mw (.q(Rdst_mw), .d(Rdst_em), .we(mem_en), .clk(clk), .rst(rst));
   Register #(1) mem_active (.q(active_wb), .d(active_mem), .we(1'b1), .clk(clk), .rst(rst));
	Register #(32) InstWord_reg_mw (.q(InstWord_mw), .d(InstWord_ex), .we(mem_en), .clk(clk), .rst(rst));


//------------------------------------------------------------------------------------------------------------  
   // writeback
   // storage to register file
   assign RWrEn = ((opcode_mw == `OPCODE_COMPUTE) || // R-type
      (opcode_mw == `OPCODE_COMPUTE_I) || // immediates
      (opcode_mw == `OPCODE_LOAD) || //loads
      (opcode_mw == `OPCODE_JAL) || // jal
      (opcode_mw == `OPCODE_JALR) || //jalr
      (opcode_mw == `OPCODE_LUI) || // lui
      (opcode_mw == `OPCODE_AUIPC) && (Rdst != 5'b00000));  // auipc
   assign DataWordSized = (funct3_mw == 3'b000)? {{24{DataWord_mw[7]}}, DataWord_mw[7:0]} :     //lb
                           (funct3_mw == 3'b001)? {{16{DataWord_mw[15]}}, DataWord_mw[15:0]} :  //lh
                           (funct3_mw == 3'b010)? DataWord_mw :                              //lw
                           (funct3_mw == 3'b100)? {{24{1'b0}}, DataWord_mw[7:0]} :           //lbu
	   			            {{16{1'b0}}, DataWord_mw[15:0]};            //lhu

   assign RWrdata = (opcode_mw == `OPCODE_COMPUTE)? alu_out_mw :
	                  (opcode_mw == `OPCODE_COMPUTE_I) ? alu_out_mw :
                     (opcode_mw == `OPCODE_LOAD)? DataWordSized :
                     ((opcode_mw == `OPCODE_JAL) || (opcode_mw == `OPCODE_JALR))? PC_mw + 4 :
                     (opcode_mw == `OPCODE_LUI)? imm_u_mw : PC_mw + imm_u_mw;


   assign NPC = (active_ex && branch_case)? PC_de + imm_b_de:
                  (active_ex && jal_case)? PC_de + imm_j_de:
                  (active_ex && jalr_case)? Rdata1_de + imm_i_de:
                  PC_Plus_4;

endmodule // SingleCycleCPU

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
