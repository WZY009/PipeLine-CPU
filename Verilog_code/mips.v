
module mips( );
    reg clk, reset;
         
    initial begin
        $readmemh( "Test_Instr.txt", IM.IMem ) ; 
        $monitor("PC = 0x%8X, IR = 0x%8X", PC.oldpc, IM.Out );   

        clk = 1 ;
        reset = 0 ;
        #5 reset = 1 ;
        #20 reset = 0 ;
    end
    
    always #50 clk = ~clk;
    parameter RA_ADDRESS = 5'b11111;

    //module of PC
    //input
    wire NotKeep_current_PC;//We need keep PC constant so as to stall
    wire [1:0]  PC_sel;//Decide the final input of PC
    //output
    wire [31:0] old_PC;//current PC

    //module of npc
    wire [31:0] new_PC; 
    wire Whether_branch;

    //module of im
    wire [31:0] Instrl;

    //module of IF_ID
    //input
    wire flush;
    wire IF_ID_keep;//you have to pay attention! Because IF_ID_keep is use for keep the IF/ID constant so as to stall
    //output
    wire [31:0] NextPC_id;
    wire [31:0] Instruction_id;

    //module of extend
    wire Ctrl_extend;
    wire [31:0] ext_out;

    //module of gpr
    wire [1:0]  MemtoReg;
    wire [31:0] RegWriteData_wb;
    wire [1:0]  RegDst;
    wire [4:0]  RegWriteAddress_ex;
    wire RegWrite;
    wire [31:0] Out1;
    wire [31:0] Out2;

    //module of ctrl
    wire ALUSrcA;
    wire ALUSrcB;
    wire [4:0]  ALUCode;
    wire [31:0] ALU_out;
    wire Whether_jump;

    //module of ID_EX
    wire stall;
    wire [1:0] WB_MemtoReg_ex;
    wire WB_RegWrite_ex;
    wire MEM_MemWrite_ex;
    wire MEM_MemRead_ex;
    wire ALUSrcA_ex;
    wire ALUSrcB_ex;
    wire [4:0] EX_ALUCode_ex;
    wire [1:0] EX_RegDst_ex;
    wire [31:0] PC_ex;
    wire [31:0] Imm_ex;
    wire [4:0] RsAddress_ex;
    wire [4:0] RtAddress_ex;
    wire [4:0] RdAddress_ex;
    wire [31:0] RealOut1_ex;
    wire [31:0] RealOut2_ex;

    //these two signals decide the input signal of ALU from the signal of shamt or RealOut1 and from the immediate or RealOut2  (Attention!!! the selecting function is not fulfilled)
    wire [31:0] RealOut1;
    wire [31:0] RealOut2;

    //the final input signal of ALU
    wire [31:0] Cal1;
    wire [31:0] Cal2;



    //module of Forward Unit decides the input signal of RealOut1 and RealOut2 from the Out1 or RegWtireAddress_wb or ALUResult_mem 
    wire[1:0] ForwardA;
    wire[1:0] ForwardB;
    //these two signals decide the input signal of Hazard Detector from Out1, ALUResult_mem or ALUOut
    wire[1:0] ForwardC;
    wire[1:0] ForwardD;
    wire[31:0] IDRealRs;//When we use BNE or BEQ, we need to compare the values of two registers but we are not sure the value has been written into Rt or Rs,so these two signals is the final result of data resource in the comparison
    wire[31:0] IDRealRt;

    //EX_MEM
    wire [1:0] WB_MemtoReg_mem;//just play a role in the next data register 
    wire WB_RegWrite_mem; //just play a role in the next data register 
    wire MEM_MemWrite_mem;
    wire MEM_MemRead_mem;
    wire [4:0] RegWriteAddress_mem;
    wire [31:0] PC_mem;
    wire [31:0] ALUResult_mem;
    wire [31:0] MemWriteData_mem;

    //dm
    wire [31:0] dm_data_out;
    wire MemWrite;
    wire MemRead;

    //MEM_WB
    wire [1:0] WB_MemtoReg_wb;
    wire WB_RegWrite_wb;
    wire [4:0] RegWriteAddress_wb;
    wire [31:0] PC_wb;
    wire [31:0] ALUResult_wb;
    wire [31:0] MemOut_wb;

    //WBtoID
    wire ForwardE,ForwardF;
    wire [31:0] FinalOut1,FinalOut2;

    //decompose Instruction
    wire[5:0] op;
    wire[5:0] funct;
    wire [4:0] RsAddress_id;
    wire [4:0] RtAddress_id;
    wire [4:0] RdAddress_id;

    assign op = Instruction_id[31:26];
    assign funct = Instruction_id[5:0];
    assign RsAddress_id = Instruction_id[25:21];
    assign RtAddress_id = Instruction_id[20:16];
    assign RdAddress_id = Instruction_id[15:11];
    

    im  IM(old_PC[11:2],Instrl);
    npc NPC(IDRealRs,old_PC,NextPC_id+32'd4,Instruction_id,Whether_branch,PC_sel,new_PC);
    pc  PC(new_PC,clk,reset,NotKeep_current_PC,old_PC);

    //IF to ID register, the Flush function being integrated
    IF_ID if_id(clk,reset,flush,IF_ID_keep,old_PC,Instrl,NextPC_id,Instruction_id);

    

    //Read and write Registers
    gpr GRF(clk,reset,RsAddress_id,RtAddress_id,RegWriteAddress_wb,RegWriteData_wb,WB_RegWrite_wb,Out1,Out2);
    //Produce the signals according to op and funct used in other elements
    ctrl CTRL(op,funct,RegDst,ALUSrcA,ALUSrcB,MemRead,RegWrite,MemWrite,MemtoReg,PC_sel,Ctrl_extend,ALUCode,Whether_jump);

    //Forward the data for ID to judge Whether_branch
    //Opps!!!!!!!!!!!there are some dangerous factors.But the details are presented in the "Forwarding.v"
    MuxOne_out_of_three IDRealRS(Out1,ALUResult_mem,ALU_out,ForwardC,IDRealRs);
    MuxOne_out_of_three IDRealRT(Out2,ALUResult_mem,ALU_out,ForwardD,IDRealRt);

    //Extend the Immediate
    extend EXTEND(Ctrl_extend,Instruction_id[15:0],ext_out);

    //Choose which register (rt, rd, $ra) is the real one to write back
    MuxOne_out_of_three REGDST(RtAddress_ex,RdAddress_ex,RA_ADDRESS,EX_RegDst_ex,RegWriteAddress_ex);

    //Provide the real data output of the Register
    MuxOne_out_of_two writebackGrf_muxA(ForwardE,Out1,RegWriteData_wb,FinalOut1);
    MuxOne_out_of_two writebackGrf_muxB(ForwardF,Out2,RegWriteData_wb,FinalOut2);
    
    //A small ALU is set to test whether the condition is satisfied to Whether_branch
    ZeroTest zeroTest(IDRealRs,IDRealRt,ALUCode,Whether_branch);

    //ID to EX register, with Stall function integrated
    ID_EX id_ex(clk,rst,stall,MemtoReg,RegWrite,MemWrite,MemRead,ALUCode,ALUSrcA,ALUSrcB,RegDst,RsAddress_id,RtAddress_id,RdAddress_id,NextPC_id,ext_out,FinalOut1,FinalOut2,WB_MemtoReg_ex,WB_RegWrite_ex,MEM_MemWrite_ex,MEM_MemRead_ex,EX_ALUCode_ex,ALUSrcA_ex,ALUSrcB_ex,EX_RegDst_ex,RsAddress_ex,RtAddress_ex,RdAddress_ex,PC_ex,Imm_ex,RealOut1_ex,RealOut2_ex);


    //decide whether we should stall or flush the pipeline
    //when we need to stall the pipeline(lw), we need to let NotKeep_current_PC and IF_ID_keep become 1. PS:NotKeep_current_PC always keeps equal to IF_ID_keep
    //when we need to flush the pipline(beq/bne/j/jal), we need to let Whether_jump become 1
    HazardInspector hazardinspector(reset,Whether_branch,Whether_jump,RsAddress_id,RtAddress_id,RtAddress_ex,MEM_MemRead_ex,RegWriteAddress_ex,NotKeep_current_PC,IF_ID_keep,stall,flush);
    
    //Detect which data should be the input of ALU(ForwardA&B,just for data adventure), the control adventure(ForwardC&D) and solve the problem of read and write conflict(ForwardE&F)
    Forwarding forwarding(WB_RegWrite_mem,WB_RegWrite_wb,WB_RegWrite_ex,RegWriteAddress_mem,RegWriteAddress_wb,RegWriteAddress_ex,RsAddress_ex,RtAddress_ex,RsAddress_id,RtAddress_id,ForwardA,ForwardB,ForwardC,ForwardD,ForwardE,ForwardF);

    //Choose from the data (Register, Memory, ALUResult)
    MuxOne_out_of_three alua(RealOut1_ex,RegWriteData_wb,ALUResult_mem,ForwardA,RealOut1);//Forward choose A
    MuxOne_out_of_three alub(RealOut2_ex,RegWriteData_wb,ALUResult_mem,ForwardB,RealOut2);//Forward choose B
    //Choose from data in Register or in Immediate 
    //There is an unmatched things between the image and the code. You can notice that there is one out of four MUX in the image while the code does not have this module. I use a one out of two MUX and a one out of three MUX to achieve this goal. 
    MuxOne_out_of_two aluSrcA(ALUSrcA_ex,RealOut1,{27'b0, Imm_ex[10:6]},Cal1);
    MuxOne_out_of_two aluSrcB(ALUSrcB_ex,RealOut2,Imm_ex,Cal2);
    //ALU
    alu ALU(Cal1, Cal2, EX_ALUCode_ex,ALU_out,);

    //EX to MEM register
    EX_MEM ex_mem(clk,reset,WB_MemtoReg_ex,WB_RegWrite_ex,MEM_MemWrite_ex,MEM_MemRead_ex,RegWriteAddress_ex,PC_ex,ALU_out,RealOut2,WB_MemtoReg_mem,WB_RegWrite_mem,MEM_MemWrite_mem,MEM_MemRead_mem,RegWriteAddress_mem,PC_mem,ALUResult_mem,MemWriteData_mem);


    dm DM(ALUResult_mem,MemWriteData_mem,MEM_MemWrite_mem,MEM_MemRead_mem,clk,reset,dm_data_out);//this module is given, so we do not need to foucus on it.
    //MEM to WB register
    MEM_WB mem_wb(clk,reset,WB_MemtoReg_mem,WB_RegWrite_mem,RegWriteAddress_mem,PC_mem,ALUResult_mem,dm_data_out,WB_MemtoReg_wb,WB_RegWrite_wb,RegWriteAddress_wb,PC_wb,ALUResult_wb,MemOut_wb);
    
    //Choose from the data (ALUResult, Memory, old_PC) to write to register
    //DatatoReg_mux datatoreg(WB_MemtoReg_wb,ALUResult_wb,MemOut_wb,PC_wb,RegWriteData_wb);
    MuxOne_out_of_three datatoreg(ALUResult_wb,MemOut_wb,PC_wb+32'd4,WB_MemtoReg_wb,RegWriteData_wb);
endmodule
