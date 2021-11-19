`include "opcodes.v" 
module ALU(A, B, ALUControl, ALUOut);
   input [15:0] A;
   input [15:0] B;
   input [5:0] ALUControl;
   output reg [15:0] ALUOut;                  

   always @(*)
   begin
      case(ALUControl)
        6'd0: // Addition
          ALUOut = A+B; 
        6'd1: // Subtraction
          ALUOut = A-B;
        6'd2: // AND
          ALUOut = A & B;
        6'd3: // OR
          ALUOut = A | B;
        6'd4: // NOT
          ALUOut = ~(A);
        6'd5: // TCP
          ALUOut = ~(A)+1;
        6'd6: // Logical left shift
          ALUOut = A<<1;
        6'd7: // Logical right shift
          ALUOut = A>>1;
        6'd8:
          ALUOut = {B[7:0],8'h00};
        6'd9:
          ALUOut = (A == B) ? 0 : 1;
        6'd10:
          ALUOut = (A == B) ? 1 : 0;
        6'd11:
          ALUOut = ($signed(A) > 0) ? 1 : 0;
        6'd12:
          ALUOut = ($signed(A) < 0) ? 1 : 0;
        6'd13:
          ALUOut = A;
        6'd28:
          ALUOut = A;
        6'd29:
          ALUOut = 1;
      endcase
   end
endmodule