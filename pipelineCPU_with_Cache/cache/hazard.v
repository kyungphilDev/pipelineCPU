`include "opcodes.v"

module hazard_detect(ID_RS, ID_RT, EX_Mem_Read, EX_Reg_Write, EX_RS, EX_RT, is_stall, IFIDWrite, IDEXWrite, EXMEMWrite,PCWrite, ic_enable, dc_enable);
  input [1:0] ID_RS;
  input [1:0] ID_RT;
  input [1:0] EX_RS;
  input [1:0] EX_RT;
  input EX_Mem_Read;
  input EX_Reg_Write;
  input ic_enable;
  input dc_enable;
  input IDEXWrite;
  input EXMEMWrite;

  output reg is_stall;
	output reg PCWrite;
  output reg IFIDWrite;

  assign IDEXWrite = dc_enable;
  assign EXMEMWrite = dc_enable;

  always @(*) begin //////////maybe?
    if(((EX_Mem_Read ) && ((EX_RT == ID_RS) || (EX_RT == ID_RT))) || !ic_enable || !dc_enable) begin
      is_stall = 1'b1;
      IFIDWrite = 1'b1;
      PCWrite = 1'b1;
    end
    else begin
      is_stall = 1'b0;
      IFIDWrite = 1'b0;
      PCWrite = 1'b0;
    end
  end
endmodule