`include "opcodes.v" 

module register_file(read_out1,read_out2, read1, read2,  dest, write_data, reg_write, clk, reset_n, WWD, output_port);
  input clk, reset_n;
	input [1:0] read1;	// rs
	input [1:0] read2;	// rt
	input [1:0] dest;	// rd
  input reg_write;
  input [`WORD_SIZE-1:0] write_data;
  input WWD;
	output [`WORD_SIZE-1:0] read_out1;
	output [`WORD_SIZE-1:0] read_out2;
  output reg [`WORD_SIZE-1:0] output_port;

  reg [`WORD_SIZE-1:0] register[`NUM_REGS-1:0];
  integer i;
  
  //initialize reg files
  initial begin
      for(i=0; i<`NUM_REGS;i=i+1) begin
          register[i] = `WORD_SIZE'b0;
      end
  end

  //DataForwarding when dist = 3
  assign read_out1 = (reg_write && dest == read1) ? write_data : register[read1];
  assign read_out2 = (reg_write && dest == read2) ? write_data : register[read2];

  always @(posedge clk) begin
    if(!WWD) begin
      if(reg_write) begin
       register[dest] <= write_data;
      end
    end
    else begin
      output_port <= write_data; //WWD
    end
  end
endmodule

module IFID_Reg(clk, PC_Plus_1_In, PC_Plus_1_Out, Instr_In, Instr_Out, IFID_Write, Predict_In, Predict_Out);
  input clk;
  input [`WORD_SIZE-1:0] PC_Plus_1_In;
  input [`WORD_SIZE-1:0] Instr_In;
  input IFID_Write;
  input Predict_In;
  output reg [`WORD_SIZE-1:0] PC_Plus_1_Out;
  output reg [`WORD_SIZE-1:0] Instr_Out;
  output reg Predict_Out;

  always @(posedge clk) begin
    if(!IFID_Write) begin
      PC_Plus_1_Out <= PC_Plus_1_In;
      Instr_Out <= Instr_In;
      Predict_Out <= Predict_In;
    end
  end
endmodule

module IDEX_Reg(ID_JXX, EX_JXX, ID_JRL_JPR, EX_JRL_JPR, ID_BranchINFO, EX_BranchINFO, clk, PC_Plus_1_In, Control_In, Predict_In, ID_read_out1,  ID_read_out2,  Extended_Imm_In,  ID_RS,  ID_RT,  ID_RD,  PC_Plus_1_Out,  Control_Out,  Predict_Out,  RegDst,  ALUOp, ALUSrc, EX_read_out1, EX_read_out2, Extended_Imm_Out, EX_RS, EX_RT, EX_RD);
  
  input wire clk;
  input [`WORD_SIZE-1:0] ID_read_out1;
  input [`WORD_SIZE-1:0] ID_read_out2;
  input [`WORD_SIZE-1:0] PC_Plus_1_In;
  input [`WORD_SIZE-1:0] Control_In;
  input [`WORD_SIZE-1:0] Extended_Imm_In;
  input Predict_In;
  
  input ID_JRL_JPR;
  output reg EX_JRL_JPR;
  input [2:0] ID_BranchINFO;
  output reg [2:0] EX_BranchINFO;
  input ID_JXX;
  output reg EX_JXX;


  //For DataForwarding var
  input [1:0] ID_RS;
  input [1:0] ID_RT;
  input [1:0] ID_RD;

  output reg [`WORD_SIZE-1:0] EX_read_out1;
  output reg [`WORD_SIZE-1:0] EX_read_out2;
  output reg [`WORD_SIZE-1:0] Extended_Imm_Out;
  //For DataForwarding var
  output reg [1:0] EX_RS;
  output reg [1:0] EX_RT;
  output reg [1:0] EX_RD;

  output reg [6:0] Control_Out;
  output reg [1:0] RegDst; // choose rt or rd
  output reg [1:0] ALUOp;
  output reg ALUSrc;
  output reg [`WORD_SIZE-1:0] PC_Plus_1_Out;
  output reg Predict_Out;

  always @(posedge clk) begin
    EX_read_out1 <= ID_read_out1;
    EX_read_out2 <= ID_read_out2;
    PC_Plus_1_Out <= PC_Plus_1_In;
    RegDst <= Control_In[4:3];
    ALUOp <= Control_In[2:1];
    ALUSrc <= Control_In[0];
    Extended_Imm_Out <= Extended_Imm_In;
    Control_Out <= Control_In[15:9];
    Predict_Out <= Predict_In;
    EX_BranchINFO <= ID_BranchINFO;
    EX_JRL_JPR <= ID_JRL_JPR;
    EX_JXX <= ID_JXX;
    EX_RS <= ID_RS;
    EX_RT <= ID_RT;
    EX_RD <= ID_RD;
  end
endmodule


module EXMEM_Reg(clk, Control_In, ALU_Result_In, Write_Data_In, EX_RD, Flush_In, Control_Out, mem_read, mem_write, ALU_Result_Out, Write_Data_Out, MEM_RD, Flush_Out);
  input wire clk;
  input [6:0] Control_In;
  input [`WORD_SIZE-1:0] ALU_Result_In;
  input [`WORD_SIZE-1:0] Write_Data_In;
  input [1:0] EX_RD;
  input Flush_In;
  output reg [4:0] Control_Out;
  output reg mem_read;
  output reg mem_write;
  output reg [`WORD_SIZE-1:0] ALU_Result_Out;
  output reg [`WORD_SIZE-1:0] Write_Data_Out;
  output reg [1:0] MEM_RD;
  output reg Flush_Out;

  always @(posedge clk) begin
    ALU_Result_Out <= ALU_Result_In;
    Write_Data_Out <= Write_Data_In;
    MEM_RD <= EX_RD;

    Flush_Out <= Flush_In;

    Control_Out <= Control_In[6:2];
    mem_read <= Control_In[1];
    mem_write <= Control_In[0];
  end
endmodule

module MEMWB_Reg(clk, Control_In, MEM_ALU_Out, read_data_In, MEM_RD, WB_reg_write, WB_MemToReg, WB_ALU_Out, read_data_Out, WB_RD, num_inst_plus, WWD, HLT);
  input wire clk;
  input [4:0] Control_In;
  input [`WORD_SIZE-1:0] MEM_ALU_Out;
  input [`WORD_SIZE-1:0] read_data_In;
  input [1:0] MEM_RD;
  output reg WB_reg_write;
  output reg WB_MemToReg;
  output reg [`WORD_SIZE-1:0] WB_ALU_Out;
  output reg [`WORD_SIZE-1:0] read_data_Out;
  output reg [1:0] WB_RD;
  output reg num_inst_plus;
  output reg WWD;
  output reg HLT;


  always @(posedge clk) begin

    WB_reg_write <= Control_In[1];
    WB_MemToReg <= Control_In[0];

    WB_ALU_Out <= MEM_ALU_Out;
    read_data_Out <= read_data_In;
    WB_RD <= MEM_RD;

    //Not Stall
    if(Control_In[2]) begin
      num_inst_plus <= 1;
    end
    else begin
      num_inst_plus <= 0;
    end

    WWD <= Control_In[3];
    HLT <= Control_In[4];
  end
endmodule