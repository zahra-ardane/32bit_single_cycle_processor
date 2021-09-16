module DataMemory(address,memRead,memWrite,readData,clock,reset,writeData);
input[31:0]address;
input memRead,memWrite,clock,reset;
output[31:0]readData;
input[31:0] writeData;
reg [127:0] dataMem[31:0];
wire[5:0] shifted;
assign shifted=address[6:2];
integer i;
assign readData = (memRead) ? dataMem[shifted] : 32'bx;
always @(posedge clock or posedge reset)
	begin
		if (reset == 1'b1) 
			begin
				for (i=0; i<127; i=i+1) begin
					dataMem[i] = 32'b0;
				end
			end
		
			if (memWrite) dataMem[shifted] = writeData;

	end
endmodule
module RegisterFile(readRegister1,readRegister2,writeRegister,writeData,readData1,readData2,clock,reset,regWrite);
input[4:0]readRegister1,readRegister2,writeRegister;
input clock,regWrite,reset;
input [31:0] writeData;
output[31:0]readData1,readData2;
reg [31:0]regFile[31:0];
integer i;

assign readData1 = regFile[readRegister1];
assign readData2 = regFile[readRegister2];

always @(posedge clock or posedge reset) 
	begin
		if (reset==1'b1)
		begin
			for (i=0; i<32; i=i+1) 
			begin
				regFile[i] = 32'b0;
			end
		end 
		
		 if (regWrite == 1'b1) regFile[writeRegister] = writeData; 
	end
	


endmodule

module PCounter(clock, reset, PCIn, PCOut);

input clock, reset;
input [31:0] PCIn;
output reg [31:0] PCOut;
always @(posedge clock or posedge reset) begin
		if (reset == 1) 
			PCOut <= 0;

		else 
			PCOut <= PCIn; 
	end
endmodule

module ALUControl(ALUOp, Function, opCode,out);
input [1:0] ALUOp;
input [5:0] Function;
input [5:0] opCode;
output reg [3:0] out;

always @(ALUOp, Function,opCode) begin
if(ALUOp == 0)
  out <= 2;   
	else if(ALUOp == 1)
		out <= 6;		
	else if(ALUOp==2)
		case(Function)
			32: out <= 2; //add
			34: out <= 6; //subtract		
			36: out <= 0; //and	
			37: out <= 1; //or	
		        //8:  out <= 2; //addi
			42: out <= 7; //slt
			//10: out <= 7;//slti
 			//12: out <= 0; //andi
			//13: out <= 1; //ori	
			default: out <= 15;
		endcase
	else if(ALUOp==3)
		case(opCode)
			//32: out <= 2; //add
			//34: out <= 6; //subtract		
			//36: out <= 0; //and	
			//37: out <= 1; //or	
		        8:  out <= 2; //addi
			//42: out <= 7; //slt
			10: out <= 7;//slti
 			12: out <= 0; //andi
			13: out <= 1; //ori	
			default: out <= 15;
		endcase




	end
	


endmodule
module Sign_Extension (sign_in, sign_out);
	input [15:0] sign_in;
	output [31:0] sign_out;
	assign sign_out[15:0]=sign_in[15:0];
	assign sign_out[31:16]=sign_in[15]?16'b1111_1111_1111_1111:16'b0;
endmodule

module instructionMemory(address,instruction,reset);
input reset;
input[31:0] address;
output [31:0] instruction;
wire[5:0] shifted;
reg [127:0] instructionMem[31:0];
assign shifted=address[6:2];
assign instruction = instructionMem[shifted];
integer i;

always @(posedge reset) begin
for (i=0; i<128; i=i+1) begin
		instructionMem[i] = 32'b0;
	end
instructionMem[0]=32'b00100000000010010000000000110111; //addi $t1, $zero, 55
instructionMem[1]=32'b10101100000010010000000000001000; //sw $t1, 8($zero)
instructionMem[2]=32'b00100000000010000000000000110111; //addi $t0, $zero, 55
instructionMem[3]=32'b00000001000010011000100000100101; //or s1, t1, t0
instructionMem[4]=32'b00000001001010001001000000100010; //sub s2, t1, t0
instructionMem[5]=32'b00010010001100100000000000000011; //beq $s1, $s2, label
instructionMem[6]=32'b10001100000100010000000000000100; //lw $s1, 4($zero)
instructionMem[7]=32'b00110010001100100000000001001000; //andi $s2, $s1, 72
instructionMem[8]=32'b10101100000100000000000000000100; //sw $s0, 4($zero)
instructionMem[9]=32'b10101100000100000000000000000100; //sw $s0, 4($zero)(label)
instructionMem[10]=32'b00001000000000000000000000001110; //j Last
instructionMem[11]=32'b10101100000100000000000000000100; //sw $s0, 4($zero)
instructionMem[12]=32'b10101100000100000000000000000100; //sw $s0, 4($zero)
instructionMem[13]=32'b10101100000100000000000000000100; //sw $s0, 4($zero)
instructionMem[14]=32'b00100010001100010000000000101011;//addi $s1,$s1, 43(Last)
instructionMem[15]=32'b00010010001010010000000000000001; //beq &s1,$t1, label2
instructionMem[16]=32'b10101100000100000000000000000100; //sw $s0, 4($zero)
instructionMem[17]=32'b00000010010100011010000000101010; //slt $s4, $s2, $s1(label2)
instructionMem[18]=32'b00101010010011010000000000001100; //slti $t5, $s2, 12
instructionMem[19]=32'b00000001101010010110100000100010; //sub $t5, $t5, $t1
instructionMem[20]=32'b00110010001100100000000001001000; //andi $s2, $s1, 48 
instructionMem[21]=32'b00000001000010011000000000100101; //or $s0, $t0, $t1
instructionMem[22]=32'b00010110000010000000000000000001; //bne $s0, $t0, next
instructionMem[23]=32'b00001000000000000000000000011111; //j EXIT
instructionMem[24]=32'b00110101000010010000000000000001; //ori $t1, $t0,1(next) 
instructionMem[25]=32'b00001000000000000000000000011111; //j EXIT


end

endmodule
module AddALU(data1,data2,out);
input[31:0]data1,data2;
output [31:0]out;
assign out=data1+data2;

endmodule
module Control(opCode, regDst, jump, branch, memRead, memtoReg, ALUOp, memWrite, ALUSrc, regWrite);
input [5:0] opCode;
output reg regDst, jump, branch, memRead, memtoReg, memWrite, ALUSrc, regWrite;
output reg [1:0] ALUOp;
always @(*) begin
if(opCode == 6'b101011) begin //sw
  regDst    <= 0; //X
  jump      <= 0;
  ALUSrc     <= 1;
  memtoReg  <= 0; //X
  regWrite    <= 0;
  memRead    <= 0;
  memWrite    <= 1;
  branch    <= 0;
  ALUOp      <= 2'b00;
end 
else if (opCode == 6'b100011) begin //lw
  regDst    <= 0;
  jump      <= 0;
  ALUSrc     <= 1;
  memtoReg  <= 1;
  regWrite    <= 1;
  memRead    <= 1;
  memWrite    <= 0;
  branch    <= 0;
  ALUOp      <= 2'b00;
end
else if (opCode == 6'b001000 || opCode == 6'b001010 || opCode == 6'b001100 || opCode == 6'b001101) begin //addi,slti,andi,ori
  regDst    <= 0;
  jump      <= 0;
  ALUSrc     <= 1;
  memtoReg  <= 0;
  regWrite    <= 1;
  memRead    <= 0;
  memWrite    <= 0;
  branch    <= 0;
  ALUOp      <= 2'b11; //?
end 
else if (opCode == 6'b000100 || opCode == 6'b000101) begin //beq,bne
  regDst    <= 0; //X
  jump      <= 0;
  ALUSrc     <= 0;
  memtoReg  <= 0; //X
  regWrite    <= 0;
  memRead    <= 0;
  memWrite    <= 0;
  branch    <= 1;
  ALUOp      <= 2'b01; //?
end 
else if (opCode == 6'b000000) begin //add, sub, or, and, slt
  regDst    <= 1;
  jump      <= 0;
  ALUSrc     <= 0;
  memtoReg  <= 0;
  regWrite    <= 1;
  memRead    <= 0;
  memWrite    <= 0;
  branch    <= 0;
  ALUOp      <= 2'b10;
end 
else if (opCode == 6'b000010) begin //j
  regDst    <= 0;
  jump      <= 1;
  ALUSrc     <= 0;
  memtoReg  <= 0;
  regWrite    <= 0;
  memRead    <= 0;
  memWrite    <= 0;
  branch    <= 0;
  ALUOp      <= 2'b00; //X
end 
  end

endmodule


module instructionParser(instruction,opCode,rs,rt,rd,shamt,funct,address,target);
  input [31:0] instruction;
  output [5:0] opCode;
  output [4:0] rs;
  output [4:0] rt;
  output [4:0] rd;
  output [4:0] shamt;
  output [5:0] funct;
  output [15:0] address; //I-type
  output [25:0] target; //J-type

  assign opCode = instruction[31:26];
  assign rs = instruction[25:21];
  assign rt = instruction[20:16];
  assign rd = instruction[15:11];
  assign shamt = instruction[10:6];
  assign funct = instruction[5:0];
  assign address[15:0] = {rd,shamt,funct};
  assign target[25:0] = {rs,rt,rd,shamt,funct};

endmodule
module ALU (in0, in1, alu_out, zero, control_signal);
  input [31:0] in0, in1;
  output reg [31:0] alu_out;
  output reg zero;
  input [3:0] control_signal;
  always @ (control_signal or in0 or in1) begin
    case (control_signal)
      4'b0000: begin //and,andi
         zero <= 0;
         alu_out <= in0 & in1; 
      end 
      4'b0001: begin  //or,ori
        zero <= 0; 
        alu_out <= in0 | in1; 
      end
      4'b0010: begin //add,addi
        zero <= 0; 
        alu_out <= in0 + in1; 
      end
      4'b0110: begin //sub
        if(in0 == in1) 
          zero <= 1; 
        else 
          zero <= 0; 
        alu_out <= in0 - in1; 
      end
      4'b0111: begin //slt,slti
        zero <= 0; 
        if(in0 > in1)
          alu_out <= 32'b1;
        else
          alu_out <= 32'b0;
      end
    endcase
  end
endmodule
module Shift_Left_2_Branch(shift_in,shift_out);
  input [31:0] shift_in;
  output [31:0] shift_out;
  assign shift_out[31:0] = shift_in[31:0] << 2;
  //assign shift_out[31:0]={shift_in[29:0],2'b00};
endmodule

module Shift_Left_2_Jump (shift_in, shift_out);
  input [25:0] shift_in;
  output [27:0] shift_out;
  assign shift_out[27:0]={shift_in[25:0],2'b00};
endmodule

module Jump_Address(inst28bit,pc,jumpAddress);
input [27:0] inst28bit;
input [31:0]pc;
output [31:0] jumpAddress;
//assign  jumpAddress[27:0]=inst28bit[27:0];
//assign jumpAddress[31:28]=pc[31:28];
assign jumpAddress[31:0]={pc[31:28],inst28bit[27:0]};

endmodule

module Mux_32_Bit (in0, in1, mux_out, control_signal);
  input [31:0] in0, in1;
  output [31:0] mux_out;
  input control_signal;
  assign mux_out = control_signal? in1: in0;
endmodule

module Mux_5_Bit (in0, in1, mux_out, control_signal);
  input [4:0] in0, in1;
  output [4:0] mux_out;
  input control_signal;
  assign mux_out = control_signal? in1: in0;
endmodule

module andGate(branch,zero,mux_control);
  input branch , zero;
  output mux_control;
  assign mux_control = branch & zero; 

endmodule

module Core(clk,reset,PCIn,PCOut,inst,
	opCode,rs,rt,rd,shamt,funct,address,target,
	regDst,jump , regWrite, ALUSrc, memtoReg, memRead, memWrite, branch,ALUOp,
	mux_5_out,readData1,readData2,
	extend32,ALUInput2,shiftedBranchAddress,ALUController,Add_ALUOut,writeData,in0_next_mux,AddALU_Out,ALUOut,readData,zero,mux_control

	,shiftedJumpAddress,jumpAddress

);
  
	input clk , reset;

  	output wire [31:0] PCIn, PCOut;
  	PCounter pc(
    		//inputs
    		.clock(clk),
    		.reset(reset),
    		.PCIn(PCIn),
    		//outputs
    		.PCOut(PCOut)  
 	 );

  	output wire [31:0] inst;
	instructionMemory inst_mem(
   		 //inputs
    		.address(PCOut),
    		.reset(reset),
    		//outputs
   		 .instruction(inst)  
 	 );

	wire [31:0] temp;
  assign temp = 32'b00000000000000000000000000000100;
  output wire [31:0] AddALU_Out;
  AddALU add_alu_1(
    //inputs
    .data1(PCOut),
    .data2(temp),
    //outputs
    .out(AddALU_Out)
  );

	output [5:0] opCode;
  	output [4:0] rs;
  	output [4:0] rt;
  	output [4:0] rd;
  	output [4:0] shamt;
  	output [5:0] funct;
  	output [15:0] address; 
  	output [25:0] target;
	instructionParser parser(
		//inputs
		.instruction(inst),
		//outputs
		.opCode(opCode),
		.rs(rs),
		.rt(rt),
		.rd(rd),
		.shamt(shamt),
		.funct(funct),
		.address(address),
		.target(target)
	);


  	output wire regDst,jump , regWrite, ALUSrc, memtoReg, memRead, memWrite, branch;
 	output wire [1:0] ALUOp;
  	Control control(
  	  	//inputs
    		.opCode(opCode),
    		//outputs
    		.regDst(regDst),
    		.jump(jump),
    		.branch(branch),
    		.memRead(memRead),
    		.memtoReg(memtoReg),
    		.ALUOp(ALUOp),
    		.memWrite(memWrite),
    		.ALUSrc(ALUSrc),
    		.regWrite(regWrite) 
 	 );

  	output wire [4:0]  mux_5_out;
  	Mux_5_Bit mux_5(
    		//inputs
    		.in0(rt),//20-16
    		.in1(rd),//15-11
   		.control_signal(regDst),
    		//outputs
    		.mux_out(mux_5_out)  
  	);

  	output wire [31:0] readData1,readData2,writeData;
  	RegisterFile reg_file(
    		//inputs
    		.clock(clk),
    		.reset(reset),
    		.readRegister1(rs),//25-21
    		.readRegister2(rt),
		.writeRegister(mux_5_out),
    		.regWrite(regWrite),
    		.writeData(writeData),
    		//outputs
    		.readData1(readData1),
    		.readData2(readData2)
  	);

  	output [31:0] extend32;
  	Sign_Extension sign_extend (
  		//inputs
  		.sign_in(address),//0-15
   		//outputs
  		.sign_out(extend32)
  	);

  	output[31:0] ALUInput2;
  	Mux_32_Bit alu_register_mux(
  		//inputs
 		.in0(readData2),
 		.in1(extend32),
 		.control_signal(ALUSrc),
 		//outputs
 		.mux_out(ALUInput2)
 	);
 
 	output[31:0] shiftedBranchAddress;
 	Shift_Left_2_Branch shift_branch (
  		//inputs
 		.shift_in(extend32),
 		//outputs
 		.shift_out(shiftedBranchAddress)
	);
  
 	output[3:0] ALUController;
 	ALUControl aluControl(
 		//inputs
 		.ALUOp(ALUOp),
 		.Function(funct),//0-5
		.opCode(opCode),
 		//outputs
 		.out(ALUController)
	);

	output wire zero;
	output wire [31:0] ALUOut;
	ALU alu(
		//inputs
		.in0(readData1),
		.in1(ALUInput2),
		.control_signal(ALUController),
		//outputs
		.alu_out(ALUOut),
		.zero(zero)
	);

	output wire [31:0] readData;
	DataMemory  data_memory(
		//inputs
		.clock(clk),
		.reset(reset),
		.address(ALUOut),
		.memWrite(memWrite),
		.memRead(memRead),
		.writeData(readData2),
		//outputs
		.readData(readData)
	);
	
  	
	Mux_32_Bit dataM_mux(
  		//inputs
 		.in0(ALUOut),
 		.in1(readData),
 		.control_signal(memtoReg),
 		//outputs
 		.mux_out(writeData)
 	);

	output wire mux_control;
	andGate and_gate(
		//inputs
		.branch(branch),
		.zero(zero),
		//outputs
		.mux_control(mux_control)
	);

	output wire [31:0] Add_ALUOut;
	AddALU add_alu_2(
		//inputs
		.data1(AddALU_Out), 
		.data2(shiftedBranchAddress),
		//outputs
		.out(Add_ALUOut)	
	);

	output wire[31:0]  in0_next_mux;
	Mux_32_Bit add_alu_mux_1(
  		//inputs
 		.in0(AddALU_Out),
 		.in1(Add_ALUOut),
 		.control_signal(mux_control),
 		//outputs
 		.mux_out(in0_next_mux)
 	);

	
   	output[27:0] shiftedJumpAddress;
  	Shift_Left_2_Jump shift_jump (
      		//inputs
     		.shift_in(target),
     		//outputs
     		.shift_out(shiftedJumpAddress)
  	);

	output[31:0]jumpAddress;
        Jump_Address j_Address(
        .inst28bit(shiftedJumpAddress),
        .pc(AddALU_Out),
	.jumpAddress(jumpAddress));


	Mux_32_Bit add_alu_mux_2(
  		//inputs
 		.in0(in0_next_mux),
 		.in1(jumpAddress),
 		.control_signal(jump),
 		//outputs
 		.mux_out(PCIn)
 	);



endmodule
module tb_processor;
reg clk;  
reg reset;
wire [31:0] PCIn, PCOut,inst,AddALU_Out;
wire [5:0] opCode;
wire [4:0] rs;
wire [4:0] rt;
wire [4:0] rd;
wire [4:0] shamt;
wire [5:0] funct;
wire [15:0] address; 
wire [25:0] target;
wire regDst,jump , regWrite, ALUSrc, memtoReg, memRead, memWrite, branch,zero;
wire [1:0] ALUOp;
wire [4:0]  mux_5_out;
wire [31:0] readData1,readData2,writeData;
wire[3:0] ALUController;
wire[27:0] shiftedJumpAddress;
wire [31:0] extend32,ALUInput2,shiftedBranchAddress,ALUOut,readData,Add_ALUOut,in0_next_mux,jumpAddress
;

Core core(.clk(clk),
.reset(reset),
.PCIn(PCIn),
.PCOut(PCOut),
.inst(inst),
.opCode(opCode),
.rs(rs),
.rt(rt),
.rd(rd),
.shamt(shamt),
.funct(funct),
.address(address),
.target(target),
.regDst(regDst),
.jump(jump),
.regWrite(regWrite),
.ALUSrc(ALUSrc),
.memtoReg(memtoReg),
.memRead(memRead), 
.memWrite(memWrite),
.branch(branch),
.ALUOp(ALUOp),
.mux_5_out(mux_5_out),
.readData1(readData1),
.readData2(readData2),
.extend32(extend32),
.ALUInput2(ALUInput2),
.shiftedBranchAddress(shiftedBranchAddress),
.ALUController(ALUController),
.Add_ALUOut(Add_ALUOut),
.writeData(writeData),
.in0_next_mux(in0_next_mux),
.AddALU_Out(AddALU_Out),
.ALUOut(ALUOut),
.readData(readData),
.zero(zero),
.mux_control(mux_control),
.shiftedJumpAddress(shiftedJumpAddress),
.jumpAddress(jumpAddress)
);

 initial begin  
           clk = 1;  
           forever #50 clk = ~clk;  
      end  
      initial begin  
          
           reset = 1;  
         
           #100;  
     reset = 0;  
             
      end  
 endmodule  






