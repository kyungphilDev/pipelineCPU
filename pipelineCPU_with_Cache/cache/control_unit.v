`include "opcodes.v" 

module ControlUnit(opcode, func_code, clk, reset_n, Control_Set, Branch_INFO, ID_JRL_JPR, ID_JXX);
	input [3:0] opcode;
	input [5:0] func_code;
  input clk;
	input reset_n;

  output [`WORD_SIZE-1:0] Control_Set;
  output reg [2:0] Branch_INFO;
  output reg ID_JRL_JPR;
  output reg ID_JXX;

  reg HLT;
  reg WWD;
  reg is_reset;
  reg [1:0] RegDst;
  reg [1:0] ALUOp;
  reg ALUSrc;
  reg MemRead;
  reg MemWrite;
  reg RegWrite;
  reg MemToReg;

  assign Control_Set = {HLT, WWD, is_reset, RegWrite, MemToReg, MemRead, MemWrite, 1'b0, 3'b00, RegDst, ALUOp, ALUSrc};
// {HLT, WWD, is_reset, RegWrite, MemToReg, MemRead, MemWrite, 0, 000, RegDst, ALUOp, ALUSrc};
//   15  14      13      12         11         10       9        8     7:5    4:3     2:1    0   // to check index of Control_Set
//   6    5       4       3          2        1        0       
//   4    3       2       1          0
  always @(posedge clk) begin
    if(!reset_n) begin
        HLT <= 1'b0;
        WWD <= 1'b0;
        is_reset <= 1'b0;
        RegDst <= 2'b00;
        ALUOp <= 2'b00;
        ALUSrc <= 1'b0;
        MemRead <= 1'b0;
        MemWrite <= 1'b0;
        RegWrite <= 1'b0;
        MemToReg <= 1'b0;
        Branch_INFO <= 3'b000;
    end
  end
  always @(*) begin
   //---------------RegDst---------------------
   if( (opcode == `JAL_OP) || ( (opcode == `R_Type) && (func_code == `INST_FUNC_JRL))) begin
     RegDst = 2'b10; //JAL, JRL
   end
   else if( (opcode == `R_Type) &&  (func_code != `INST_FUNC_JRL) && (func_code != `INST_FUNC_JPR) && (func_code != `INST_FUNC_HLT) ) begin
     RegDst = 2'b01;
   end
   else begin
     RegDst = 2'b00;
   end
   //---------------Branch_INFO---------------------
   if(opcode >= 0 && 3 >= opcode) begin
     Branch_INFO = {1'b1, opcode[1:0]};
   end
   else begin
     Branch_INFO = 3'b000;
   end
   //---------------ALUOP, ALUSrc, MemWrite---------------------
   if(opcode == `R_Type ) begin // R type
     if(func_code == `INST_FUNC_JPR) begin  // JPR
       ALUOp = 2'b00;
       ALUSrc = 1'b0;
       MemWrite = 1'b0;
     end
     else if(func_code == `INST_FUNC_JRL) begin  // JRL
       ALUOp = 2'b11;
       ALUSrc = 1'b0;
       MemWrite = 1'b0;
     end
     else if(func_code == `INST_FUNC_HLT) begin  // HLT
       ALUOp = 2'b00;
       ALUSrc = 1'b0;
       MemWrite = 1'b0;
     end
     else begin
       ALUOp = 2'b11;
       ALUSrc = 1'b0;          
       MemWrite = 1'b0;
     end
   end
   else if(opcode == 4'b0100 || opcode == 4'b0101 || opcode == 4'b0110 ) begin // I type
       if(opcode == `ADI_OP) begin
         ALUOp = 2'b00;
       end
       else if(opcode == `ORI_OP) begin
         ALUOp = 2'b01;
       end
       else if(opcode == `LHI_OP) begin
         ALUOp = 2'b10;
       end
       ALUSrc = 1'b1;
       MemWrite = 1'b0;
   end 
   else if(opcode == 4'b0111) begin // LW
       ALUOp = 2'b00;
       ALUSrc = 1'b1;
       MemWrite = 1'b0;
   end
   else if(opcode == 4'b1000) begin // SW
       ALUOp = 2'b00;
       ALUSrc = 1'b1;
       MemWrite = 1'b1;
   end
   else if(opcode >= 0 && 3 >= opcode) begin // Branch
       ALUOp = 2'b11;
       ALUSrc = 1'b0;
       MemWrite = 1'b0;
   end
   else if(opcode == 4'b1001) begin // JMP
       ALUOp = 2'b00;
          ALUSrc = 1'b0;
          MemWrite = 1'b0;
      end
      else if(opcode == 4'b1010) begin // JAL
          ALUOp = 2'b11;
          ALUSrc = 1'b0;
          MemWrite = 1'b0;
      end
    end
  assign HLT = (opcode === `R_Type) && (func_code === `INST_FUNC_HLT);
  assign WWD = (opcode === `WWD_OP) && (func_code === `INST_FUNC_WWD);
  assign is_reset = (opcode >= 0); // after the reset, to check whether opcode is runnging
  assign RegWrite = (opcode != `SWD_OP) && ((opcode != `BNE_OP) && (opcode != `BEQ_OP) && (opcode != `BGZ_OP) && (opcode != `BLZ_OP)) && (opcode != `JMP_OP) && ((opcode != `JPR_OP) || (func_code != `INST_FUNC_JPR)) && (func_code != `INST_FUNC_WWD);
  assign MemToReg = (opcode === `LWD_OP);
  assign MemRead = (opcode === `LWD_OP);
  assign ID_JRL_JPR = (opcode === `R_Type) && ((func_code === `INST_FUNC_JPR) || (func_code === `INST_FUNC_JRL));
  assign ID_JXX = (opcode === `R_Type) && ((func_code === `INST_FUNC_JPR) || (func_code === `INST_FUNC_JRL)) || (opcode == `JAL_OP);
endmodule
