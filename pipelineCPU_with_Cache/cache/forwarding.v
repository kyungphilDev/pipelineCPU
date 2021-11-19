module forwarding_unit(JRL, EX_RS, EX_RT, MEM_Reg_Write, MEM_RD, WB_Reg_Write, WB_RD, Forward_A, Forward_B);
  input [1:0] WB_RD;
  input [1:0] EX_RT;
  input [1:0] MEM_RD;
  input [1:0] EX_RS;
  input wire JRL;
  input WB_Reg_Write;
  input MEM_Reg_Write;
  output reg [1:0] Forward_A;
  output reg [1:0] Forward_B;

  always @(*) begin
    if(MEM_Reg_Write && (MEM_RD == EX_RS)) begin
      Forward_A = 2'b10;
    end
    else if(WB_Reg_Write && (WB_RD == EX_RS)) begin 
      Forward_A = 2'b01;
    end
    else begin
      Forward_A = 2'b00;
    end
    if(MEM_Reg_Write && (MEM_RD == EX_RT)) begin
      Forward_B = 2'b10;
    end
    else if(WB_Reg_Write && (WB_RD == EX_RT)) begin 
      Forward_B = 2'b01;
    end
    else begin
      Forward_B = 2'b00;
    end
  end
endmodule
