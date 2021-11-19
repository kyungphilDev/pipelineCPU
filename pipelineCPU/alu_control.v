`include "opcodes.v" 

module ALUControlUnit(EX_BranchINFO, EX_JXX, ALUOp, func, ALUControl);

  input wire [1:0] ALUOp;
  input wire [5:0] func;
  input [2:0] EX_BranchINFO;
  input EX_JXX;

  output reg [5:0] ALUControl;

  always @(*) begin
    if(ALUOp == 2'b00) begin // add
      ALUControl = 6'd0;
    end
    if(ALUOp == 2'b01) begin // or
      ALUControl = 6'd3;
    end
    if(ALUOp == 2'b10) begin // lhi
      ALUControl = 6'd8;
    end
    //Bxx
    if(EX_BranchINFO == 3'b100) begin
      ALUControl = 6'd9;
    end
    else if(EX_BranchINFO == 3'b101) begin
      ALUControl = 6'd10;
    end
    else if(EX_BranchINFO == 3'b110) begin
      ALUControl = 6'd11;
    end
    else if(EX_BranchINFO == 3'b111) begin
      ALUControl = 6'd12;
    end      
    if(ALUOp == 2'b11) begin // R type
      if(EX_JXX) begin //JAL, JRL, JPR
        ALUControl = 6'd13;
      end
      if(!EX_JXX && !EX_BranchINFO[2]) begin
        ALUControl = func;
      end
    end
  end
endmodule
