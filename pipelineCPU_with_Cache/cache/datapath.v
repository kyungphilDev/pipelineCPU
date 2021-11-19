`include "opcodes.v"
`include "register_file.v" 
`include "alu.v"
`include "control_unit.v" 
`include "branch_predictor.v"
`include "hazard.v"

module datapath(clk, reset_n, read_m1, address1, data1, read_m2, write_m2, address2, data2, num_inst, output_port, is_halted);
	input clk;
	input reset_n;

	output read_m1;
	output [`WORD_SIZE-1:0] address1;
	output read_m2;
	output reg write_m2;
	output [`WORD_SIZE-1:0] address2;

	input [`WORD_SIZE-1:0] data1;
	inout [`WORD_SIZE-1:0] data2;

	output reg [`WORD_SIZE-1:0] num_inst;
	output [`WORD_SIZE-1:0] output_port;
	output is_halted;

	//=======--------------------------=======
	//  Data Path Variables
	//=======--------------------------=======
	wire num_inst_plus;
	//Program Counter var
	reg [`WORD_SIZE-1:0] PC_Cur;
	wire [`WORD_SIZE-1:0] PC_Next;
	wire [`WORD_SIZE-1:0] PC_Check;
	wire [`WORD_SIZE-1:0] PC_Predict;

	// IF var
	wire [`WORD_SIZE-1:0] IF_PC_Plus_1;
	wire [`WORD_SIZE-1:0] JMP_PC;
	wire IFID_Write;
	wire IF_PC_Write;
	wire PC_Src0;
	wire PC_Src1;

  	wire Is_Predict;
	wire IF_PredictResult;
	wire BranchCheckOut;
	wire [`WORD_SIZE-1:0] IF_BTBValue;
	wire [`WORD_SIZE-1:0] Predicted_PC;

 	// ID var
	wire [2:0] ID_BranchINFO;
	wire ID_JXX;
	wire ID_JRL_JPR;
	wire [`WORD_SIZE-1:0] ID_PC_Plus_1;
	wire [`WORD_SIZE-1:0] ID_Instruction;
	wire [`WORD_SIZE-1:0] ID_Extended_Imm;
	wire [7:0] ID_Imm;
	wire [1:0] ID_RS;
	wire [1:0] ID_RT;
	wire [1:0] ID_RD;
	wire [`WORD_SIZE-1:0] ID_read_out1;
	wire [`WORD_SIZE-1:0] ID_read_out2;
	wire [3:0] ID_Opcode;
	wire [5:0] ID_funcode;
	wire [`WORD_SIZE-1:0] Control_Set;
	wire [`WORD_SIZE-1:0] ID_Control_Set;
	wire Flush;
	wire ID_Flush;
	wire ID_PredictResult;
	wire Stall;

	// EX var
	wire [2:0] EX_BranchINFO;
	wire EX_JRL_JPR;
	wire [6:0] EX_Control_Set;
	wire [`WORD_SIZE-1:0] EX_PC_Plus_1;
  
	wire EX_RegWrite;
	wire EX_MemRead;
	wire [5:0] EX_ALUControl;
	wire [1:0] EX_ALU_Op;
	wire [5:0] EX_ALU_Func;
	wire [1:0] EX_RegDst;
	wire EX_ALU_Src;
	wire [1:0] EX_RS;
	wire [1:0] EX_RT;
	wire [1:0] EX_RD;
	wire [`WORD_SIZE-1:0] EX_read_out1;
	wire [`WORD_SIZE-1:0] EX_read_out2;
	wire [`WORD_SIZE-1:0] EX_Extended_Imm;
	wire [`WORD_SIZE-1:0] EX_ALU_Out;
	wire [`WORD_SIZE-1:0] Forwarding_Mux_Out1;
	wire [`WORD_SIZE-1:0] Forwarding_Mux_Out2;
	wire [`WORD_SIZE-1:0] MEM_Forwarding_Data;
	wire [`WORD_SIZE-1:0] WB_Forwarding_Data;
	wire [`WORD_SIZE-1:0] ALU_Input_A;
	wire [`WORD_SIZE-1:0] ALU_Input_B;
	wire [`WORD_SIZE-1:0] RT_Mux_Out;
	wire [1:0] ForwardA;
	wire [1:0] ForwardB;
	wire [1:0] EX_RT_Mux_Out;
	wire Flush_Instr;
	wire Verified_Branch_Result;
	wire EX_Flush;
	wire [`WORD_SIZE-1:0] EX_MUX_Control_Set;
	wire [`WORD_SIZE-1:0] Branch_Conditional_Address;
	wire [`WORD_SIZE-1:0] Return_Address;
	wire [`WORD_SIZE-1:0] Chosen_Address;
	wire IsTaken;
	wire EX_PredictResult;
	wire Predict_Check_Result;
	wire BranchSrc;

	// MEM var
	wire [4:0] MEM_Contro_Set;
	wire [`WORD_SIZE-1:0] MEM_ALU_Out;
	wire [`WORD_SIZE-1:0] MEM_write_data;
	wire [1:0] MEM_RD;
	wire MEM_mem_read;
	wire MEM_mem_write;
	wire MEM_reg_write;

	// WB var
	wire [1:0] WB_RD;
	wire [`WORD_SIZE-1:0] WB_read_data;
	wire [`WORD_SIZE-1:0] WB_ALU_Out;
	wire [`WORD_SIZE-1:0] WB_write_data;
	wire WB_RegWrite;
	wire WB_MemToReg;
 	wire WB_WWD;
  	wire WB_HLT;

	//=======--------------------------=======
	//  Cache
	//=======--------------------------=======
	wire [`WORD_SIZE-1:0] cache_data;
	wire ic_enable;

	instruction_cache ic(
	  .clk(clk), 
	  .reset_n(reset_n), 
	  .instruction(PC_Cur), 
	  .cache_data(cache_data),
	  .cache_enable(ic_enable), 
	  .mem_data(data1), 
	  .mem_read(read_m1), 
	  .mem_address(address1)
	);

	wire dc_enable;
	wire [`WORD_SIZE-1:0] dcache_data;
	data_cache dc(
	  .clk(clk), 
	  .reset_n(reset_n), 
	  .data_address(MEM_ALU_Out), 
	  .write_data(MEM_write_data),
	  .cache_data(dcache_data),
	  .cache_read(MEM_mem_read), 
	  .cache_write(MEM_mem_write), 
	  .cache_enable(dc_enable), 
	  .mem_data(data2), 
	  .mem_read(read_m2), 
	  .mem_address(address2)
	);
	
  	//=======--------------------------=======
	//  PC Stage
	//=======--------------------------=======
	//assign address1 = PC_Cur;

  
  	//PC+1 Adder
  	assign IF_PC_Plus_1 = PC_Cur + 1;

  	//=======--------------------------=======
  	//  Control Units
  	//=======--------------------------=======

	//=======--------------------------=======
  	//  Branch Prediction Units
  	//=======--------------------------=======
  	BranchCheckUnit BCU(cache_data[15:12], Is_Branch);
	assign is_BJ_type = (EX_ALUControl > 8) ? ((EX_ALUControl < 14) ? 1: 0): 1'b0;
  	branch_predictor B_Predictor(
  	  .clk(clk), 
  	  .reset_n(reset_n),
  	  .PC(IF_PC_Plus_1),
  	  .actual_PC(EX_PC_Plus_1),
  	  .Is_Taken(IsTaken),
  	  .Predict(Is_Predict),
  	  .actual_next_PC(Branch_Conditional_Address),
  	  .Predict_next_PC(IF_BTBValue),
  	  .is_BJ_type(is_BJ_type)
  	);
	assign Flush_Instr =  EX_JRL_JPR; //JPR, JRL instructions are hard to predict so flush! 
  	assign Verified_Branch_Result = ((EX_ALU_Out & EX_BranchINFO[2]) === 1'b1) ? 1'b1 : 1'b0; //verified Branch result
  	
	assign Predicted_PC = IF_BTBValue;
  	assign IF_PredictResult = ((Is_Branch === 1'b1) && (Is_Predict === 1'b1)) ? 1'b1 : 1'b0;
 	assign PC_Src1 = IF_PredictResult;
  	MUX_3 PC_PredictMux(IF_PC_Plus_1, JMP_PC, Predicted_PC, {PC_Src1, PC_Src0}, PC_Predict);
  	MUX_2 PC_NextMux(PC_Predict, PC_Check, Predict_Check_Result, PC_Next);
	  
  	//=======--------------------------=======
  	//  Data Forwarding & Hazard Detection
  	//=======--------------------------=======
  	ControlUnit controlUnit(
	    .opcode(ID_Opcode),
	    .func_code(ID_funcode),
	    .clk(clk),
	    .reset_n(reset_n),
	    .Control_Set(Control_Set),
	    .Branch_INFO(ID_BranchINFO),
	    .ID_JRL_JPR(ID_JRL_JPR),
	    .ID_JXX(ID_JXX)
	);
	ALUControlUnit aluControlUnit(EX_BranchINFO, EX_JXX, EX_ALU_Op, EX_ALU_Func, EX_ALUControl);
	hazard_detect Hazard_Detection(ID_RS, ID_RT, EX_MemRead, EX_RegWrite, EX_RS, EX_RT, Stall, IFID_Write, IDEX_Write, EXMEM_Write, PC_Write, ic_enable, dc_enable);
  	forwarding_unit forwarding_Unit(JRL, EX_RS, EX_RT, MEM_reg_write, MEM_RD, WB_RegWrite, WB_RD, ForwardA, ForwardB);
  	PredictCheckUnit PCU(EX_PredictResult, Verified_Branch_Result, Flush_Instr, BranchSrc, Predict_Check_Result);
  	MUX_2 Address_MUX(Branch_Conditional_Address, Return_Address, EX_JXX, Chosen_Address);//to check whether JAL, JRL, JPR is
  	MUX_2 CheckMux(EX_PC_Plus_1, Chosen_Address , BranchSrc, PC_Check); 
	MUX_2 EX_FlushMux({9'b0, EX_Control_Set}, 16'd0, EX_Flush, EX_MUX_Control_Set);

  	//=======--------------------------=======
  	//   IF/ID STAGE
  	//=======--------------------------=======
  	//Parsing Instruction data
  	assign ID_Opcode = ID_Instruction[15:12];
  	assign ID_RS = ID_Instruction[11:10];
  	assign ID_RT = ID_Instruction[9:8];
  	assign ID_RD = ID_Instruction[7:6];
  	assign ID_Imm = ID_Instruction[7:0];
  	assign ID_funcode = ID_Instruction[5:0];
  	wire JAL;
	wire JMP;
	assign JMP = (cache_data[15:12] === 4'd9) ? 1'b1 : 1'b0;
  	assign JAL = (cache_data[15:12] === 4'd10) ? 1'b1 : 1'b0;
	assign PC_Src0 = (JMP || JAL ) ? 1'b1 : 1'b0;
	Jump_Address_Adder JumpAddrAdder(IF_PC_Plus_1, cache_data, JMP_PC);
	IFID_Reg IFID_Register(
	    .clk(clk),
	    .PC_Plus_1_In(IF_PC_Plus_1),
	    .PC_Plus_1_Out(ID_PC_Plus_1),
	    .Instr_In(cache_data),
	    .Instr_Out(ID_Instruction),
	    .IFID_Write(IFID_Write),
	    .Predict_In(IF_PredictResult),
	    .Predict_Out(ID_PredictResult)
	); 

  	//SignExtend 8 to 16bit
  	assign ID_Extended_Imm = {
	    ID_Imm[7],
	    ID_Imm[7],
	    ID_Imm[7],
	    ID_Imm[7],
	    ID_Imm[7],
	    ID_Imm[7],
	    ID_Imm[7],
	    ID_Imm[7],
	    ID_Imm[7:0]
	 };
  	assign ID_Flush = ((Stall === 1'b1) || (Flush === 1'b1)) ? 1'b1 : 1'b0;//

  	MUX_2 ID_FlushMux(Control_Set, 16'd0, ID_Flush, ID_Control_Set);//

  	//Register File
  	register_file RF(
	    .read_out1(ID_read_out1),
	    .read_out2(ID_read_out2),
	    .read1(ID_RS),
	    .read2(ID_RT),
	    .dest(WB_RD),
	    .write_data(WB_write_data),
	    .reg_write(WB_RegWrite),
	    .clk(clk),
	    .reset_n(reset_n),
	    .WWD(WB_WWD),
	    .output_port(output_port)
	);
  	//=======--------------------------=======
  	//   ID/EX STAGE
  	//=======--------------------------=======
	IDEX_Reg IDEX_Reg(
	    .clk(clk),
	    .ID_read_out1(ID_read_out1),
	    .ID_read_out2(ID_read_out2),
	    .Extended_Imm_Out(EX_Extended_Imm),//
	    .Extended_Imm_In(ID_Extended_Imm),//
	    .ID_RS(ID_RS),
	    .ID_RT(ID_RT),
	    .ID_RD(ID_RD),
	    .IDEX_Write(IDEX_Write),
	    .PC_Plus_1_In(ID_PC_Plus_1),
	    .PC_Plus_1_Out(EX_PC_Plus_1),
	    .ID_BranchINFO(ID_BranchINFO),
	    .EX_BranchINFO(EX_BranchINFO),
	    .ID_JRL_JPR(ID_JRL_JPR),
	    .EX_JRL_JPR(EX_JRL_JPR),
	    .ID_JXX(ID_JXX),
	    .EX_JXX(EX_JXX),

	    .Control_In(ID_Control_Set),
	    .Control_Out(EX_Control_Set),
	    .Predict_In(ID_PredictResult),
	    .Predict_Out(EX_PredictResult),//
	    .RegDst(EX_RegDst),//
	    .ALUOp(EX_ALU_Op),//
	    .ALUSrc(EX_ALU_Src),//
	    .EX_read_out1(EX_read_out1),
	    .EX_read_out2(EX_read_out2),
	    .EX_RS(EX_RS),
	    .EX_RT(EX_RT),
	    .EX_RD(EX_RD)
	);


  	assign EX_ALU_Func = EX_Extended_Imm[5:0];
  	assign EX_MemRead = EX_Control_Set[1];
  	assign EX_RegWrite = EX_Control_Set[3];
  	assign is_halted = (WB_HLT === 1'b1) ? 1 : 0;

  	//branch PC adder
  	assign Branch_Conditional_Address = EX_PC_Plus_1 + EX_Extended_Imm;

  	//DataForwarding for dist=1, dist=2
  	MUX_3 Forwarding_Mux_A(EX_read_out1, WB_Forwarding_Data, MEM_Forwarding_Data, ForwardA, Forwarding_Mux_Out1);
  	MUX_3 Forwarding_Mux_B(EX_read_out2, WB_Forwarding_Data, MEM_Forwarding_Data, ForwardB, Forwarding_Mux_Out2);
  	MUX_2 ALU_Input_Mux_A(Forwarding_Mux_Out1, EX_PC_Plus_1, EX_JXX, ALU_Input_A);
  	MUX_2 ALU_Input_Mux_B(Forwarding_Mux_Out2, EX_Extended_Imm, EX_ALU_Src, ALU_Input_B);
  
  	//ALU
  	ALU EX_ALU(ALU_Input_A, ALU_Input_B, EX_ALUControl, EX_ALU_Out);
  	MUX_3 Check_RT_Mux({14'd0, EX_RT}, {14'd0, EX_RD}, {14'd0, 2'b10}, EX_RegDst, RT_Mux_Out);

  	assign EX_RT_Mux_Out = RT_Mux_Out[1:0];
  	assign Return_Address = Forwarding_Mux_Out1;
  	assign EX_Flush = (Flush === 1'b1) ? 1'b1 : 1'b0;
  	assign IsTaken = ((Verified_Branch_Result === 1'b1) && (EX_JXX === 1'b0)) ? 1'b1 : 1'b0;
  	//=======--------------------------=======
  	//   EX/MEM STAGE
  	//=======--------------------------=======
  	EXMEM_Reg EXMEM_Reg(
	    .clk(clk),
	    .Control_In(EX_MUX_Control_Set[6:0]), 
	    .Control_Out(MEM_Contro_Set), 
	    .mem_read(MEM_mem_read), 
	    .mem_write(MEM_mem_write), 
	    .EXMEM_Write(EXMEM_Write),
	    .ALU_Result_In(EX_ALU_Out), 
	    .ALU_Result_Out(MEM_ALU_Out), 
	    .Write_Data_In(Forwarding_Mux_Out2), 
	    .Write_Data_Out(MEM_write_data), 
	    .EX_RD(RT_Mux_Out[1:0]), 
	    .MEM_RD(MEM_RD), 
	    .Flush_In(Predict_Check_Result), 
	    .Flush_Out(Flush)
	);

  	//Data Memory Part
  	assign MEM_reg_write = MEM_Contro_Set[1];
  	assign data2 = (MEM_mem_write) ? MEM_write_data : 16'bz;
  	assign MEM_Forwarding_Data = MEM_ALU_Out;
  	//=======--------------------------=======
  	//   EX/MEM STAGE
  	//=======--------------------------=======
  	MEMWB_Reg MEMWB_Register(
	    .clk(clk), 
	    .Control_In(MEM_Contro_Set), 
	    .MEM_ALU_Out(MEM_ALU_Out), 
	    .WB_ALU_Out(WB_ALU_Out), 
	    .MEM_RD(MEM_RD), 
	    .WB_RD(WB_RD), 
	    .read_data_In(dcache_data), 
	    .read_data_Out(WB_read_data),
	    .WB_reg_write(WB_RegWrite), 
	    .WB_MemToReg(WB_MemToReg), 
	    .num_inst_plus(num_inst_plus),
	    .WWD(WB_WWD),
	    .HLT(WB_HLT)
  	);

  	//LD or ALU
  	MUX_2 WB_Mux(WB_ALU_Out, WB_read_data, WB_MemToReg, WB_write_data);
  	assign WB_Forwarding_Data = WB_write_data;

  	always @(posedge clk) begin
	    if(!reset_n) begin
	      	write_m2 <= 0;
	      	num_inst <= 0;
		PC_Cur <= 16'hffff;
	    end
	    else begin
		if(!PC_Write) begin
      		    PC_Cur <= PC_Next;
      		end
	      	if(num_inst_plus) begin
	            num_inst <= num_inst + 1;
	      	end
	    end
	  end


	assign write_m2 = MEM_mem_write;
endmodule

