`include "opcodes.v"

module MUX_2 (i1, i2, sel, o);
   input sel;
   input [`WORD_SIZE-1:0] i1, i2;
   output reg [`WORD_SIZE-1:0] o;

   always @ (*) begin
      case (sel)
         0: o = i1;
         1: o = i2;
      endcase
   end
endmodule

module MUX_3 (i1, i2, i3, sel, o);
   input [1:0] sel;
   input [`WORD_SIZE-1:0] i1, i2, i3;
   output reg [`WORD_SIZE-1:0] o;

   always @ (*) begin
      case (sel)
         2'b00: o = i1;
         2'b01: o = i2;
         2'b10: o = i3;
         2'b11: o = i3;
      endcase
   end
endmodule

module Jump_Address_Adder(PC, Instruction, JumpAddressOut);
  input [`WORD_SIZE-1:0] PC;
  input [`WORD_SIZE-1:0] Instruction;
  output reg [`WORD_SIZE-1:0] JumpAddressOut;

  always @(*) begin
    JumpAddressOut <= { PC[15:12], Instruction[11:0] };
  end
endmodule