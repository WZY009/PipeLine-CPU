
`include "ctrl_encode_def.v"

module alu(
    input [31:0] A,
    input [31:0] B,
    input [4:0] ALUOp,

    output reg [31:0] C,
    output reg zero
);
	

	 always @(A or B or ALUOp) begin
		case(ALUOp)
			`ALUOp_ADDU: C = A + B;           
			`ALUOp_SUBU: C = A - B;           
            `ALUOp_ADD:  C = A + B;
            `ALUOp_SUB:  C = A - B;
            `ALUOp_AND:  C = A & B;
			`ALUOp_OR:   C = A | B;
            `ALUOp_EQL:  zero = (A == B)?1'b1:1'b0;//beq
            `ALUOp_BNE:  zero = (A == B)?1'b0:1'b1;//bne
            `ALUOp_SLT:  C = ((A[31]==1&&B[31]==0)||(A[31]==B[31]&&A<B))? 32'd 1 : 32'd 0 ;//I just want to save time so the first thing I do is to compare the highest position
            `ALUOp_SLL:  C = B << A[4:0];//Oops!you have to notice that we set up an extra MUX!
            `ALUOp_SRL:  C = B >> A[4:0];
            `ALUOp_SRA:  C = $signed(B) >>> A[4:0];
            `ALUOp_LUI:  C = B << 16;
		endcase
	 end
	 
endmodule



