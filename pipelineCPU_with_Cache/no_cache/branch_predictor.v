`include "opcodes.v" 

module branch_predictor(clk, reset_n, PC, actual_PC, actual_next_PC, is_BJ_type, Is_Taken, Predict, Predict_next_PC);
  //===============---------------------============
  //  2-bit Global Prediction_Saturation Counter
  //
  //  TAG_SIZE = 8
  //  BTB_IDX_SIZE = 8
  //  BTB Table Size: 2^8 BTB entry
  //  Tag Table Size: 2^8 TAG entry
  //===============---------------------============
  input clk;
  input reset_n;
  input [`WORD_SIZE-1:0] PC; //IF_PC_Plus_1
  input [`WORD_SIZE-1:0] actual_PC; //EX_PC_Plus_1
  input is_BJ_type;
  //BHT var
  input Is_Taken;
  output reg Predict;

  //BTB var
  input [`WORD_SIZE-1:0] actual_next_PC;  //actual_PC + EX_Extendec_IMM//EX_BTB_Value //computed actual next PC from branch resolve stage
  output reg [`WORD_SIZE-1:0] Predict_next_PC; // IF_BTBValue or IF_PC_Plus_1

  //BTB Table
  reg [`BTB_IDX_SIZE-1:0] BTB_Table[2**`BTB_IDX_SIZE-1:0];

  wire [`WORD_SIZE-1:0]pc_outcome;
  assign pc_outcome = actual_PC - 1;
  //BHT, Tag Table
  reg [1:0] BHT_Table[2**`BTB_IDX_SIZE-1:0];
  reg [`TAG_SIZE-1:0] Tag_Table[2**`BTB_IDX_SIZE-1:0];
  
  //btb_idx, pc_tag
  wire [`BTB_IDX_SIZE-1:0] btb_idx;
  wire [`TAG_SIZE-1:0] pc_tag;
  assign btb_idx = PC[`BTB_IDX_SIZE-1:0];
  assign pc_tag = PC[`WORD_SIZE-1:`BTB_IDX_SIZE];
  //tag hit
  wire tag_hit;
  assign tag_hit = (Tag_Table[btb_idx] == pc_tag);

  integer i;

  always @(*) begin
  //=====-------Initialize Tables----------=====
  //  BHT_Table to 01 state weakly not taken
  //  Tag_Table to 8'hff to make all the first access miss. 
  //  all the values in BTB_Table to 0
  //=====----------------------------------=====
	if (!reset_n) begin
	    for(i=0; i<2**`BTB_IDX_SIZE; i=i+1) begin
			BTB_Table[i] = 16'h0000;
			BHT_Table[i] = 2'b01;
			Tag_Table[i] = {`TAG_SIZE{1'b1}};
       end
    end
  //predict branch prediction & BTBValue when tag matches
  	if(tag_hit) begin
  	  Predict_next_PC = {pc_tag , BTB_Table[btb_idx]};
  	  if(BHT_Table[btb_idx] === 2'b00 || BHT_Table[btb_idx] === 2'b01) begin
  	    Predict = 1'b0;
  	  end
  	  else if(BHT_Table[btb_idx] === 2'b10 || BHT_Table[btb_idx] === 2'b11) begin
  	    Predict = 1'b1;
  	  end
  	end
  end

  always @(posedge clk) begin
		//update tag
		Tag_Table[actual_PC[`BTB_IDX_SIZE-1:0]] <= actual_PC[`WORD_SIZE-1:`BTB_IDX_SIZE];
		//update BTB_Table
		BTB_Table[actual_PC[`BTB_IDX_SIZE-1:0]] <= actual_next_PC[`BTB_IDX_SIZE-1:0];
		//BHT update by Saturation counter
		if(Is_Taken) begin
			case(BHT_Table[actual_PC[`BTB_IDX_SIZE-1:0]])
				2'b00: BHT_Table[actual_PC[`BTB_IDX_SIZE-1:0]] <= 2'b01;
				2'b01: BHT_Table[actual_PC[`BTB_IDX_SIZE-1:0]] <= 2'b10;
				2'b10: BHT_Table[actual_PC[`BTB_IDX_SIZE-1:0]] <= 2'b11;
				2'b11: BHT_Table[actual_PC[`BTB_IDX_SIZE-1:0]] <= 2'b11;
			endcase
		end
		else begin
			case(BHT_Table[actual_PC[`BTB_IDX_SIZE-1:0]])
				2'b00: BHT_Table[actual_PC[`BTB_IDX_SIZE-1:0]] <= 2'b00;
				2'b01: BHT_Table[actual_PC[`BTB_IDX_SIZE-1:0]] <= 2'b00;
				2'b10: BHT_Table[actual_PC[`BTB_IDX_SIZE-1:0]] <= 2'b01;
				2'b11: BHT_Table[actual_PC[`BTB_IDX_SIZE-1:0]] <= 2'b10;
			endcase      
		end
  end
endmodule

module BranchCheckUnit(Opcode, BranchCheckOut);
  input wire [3:0] Opcode;
  output reg BranchCheckOut;

  always @(*) begin
    if(Opcode == 4'b0000 || Opcode == 4'b0001 || Opcode == 4'b0010 || Opcode == 4'b0011) begin
      BranchCheckOut = 1'b1;
    end
    else begin
      BranchCheckOut = 1'b0;
    end
  end

endmodule

module IsBJCheckUnit(opcode, is_BJ_type);
  input wire [3:0] opcode;
  output reg is_BJ_type;
  always @(*) begin
    if(opcode == `BNE_OP || opcode == `BEQ_OP || opcode == `BGZ_OP || opcode == `BLZ_OP || opcode == `JAL_OP || opcode == `JPR_OP) begin
      is_BJ_type = 1'b1;
    end
    else begin
      is_BJ_type = 1'b0;
    end
  end
endmodule

module PredictCheckUnit(PredictResultIn, Verified_Branch_Result, Flush_Instr, BranchSrc, Predict_Check_Result);
  input PredictResultIn;
  input Verified_Branch_Result;
  input Flush_Instr;
  output reg BranchSrc;
  output reg Predict_Check_Result;

  always @(*) begin
    if(Flush_Instr === 1'b1) begin
      Predict_Check_Result = 1'b1;
      BranchSrc = 1'b1;
    end
    else begin
      if((PredictResultIn === 1'b0) && (Verified_Branch_Result === 1'b0)) begin
        Predict_Check_Result = 1'b0;
        BranchSrc = 1'b0;
      end
      else if((PredictResultIn === 1'b0) && (Verified_Branch_Result === 1'b1)) begin
        Predict_Check_Result = 1'b1;
        BranchSrc = 1'b1;
      end
      else if((PredictResultIn === 1'b1) && (Verified_Branch_Result === 1'b0)) begin
        Predict_Check_Result = 1'b1;
        BranchSrc = 1'b0;
      end
      else if((PredictResultIn === 1'b1) && (Verified_Branch_Result === 1'b1)) begin
        Predict_Check_Result = 1'b0;
        BranchSrc = 1'b1;
      end
      else begin
        Predict_Check_Result = 1'b0;
        BranchSrc = 1'b0;
      end
    end
  end
endmodule