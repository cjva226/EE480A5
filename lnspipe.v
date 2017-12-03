`timescale 1ns/1ps
`define WORD		15:0
`define HALFWORD	7:0

`define REGSIZE		15:0
`define	MEMSIZE		65535:0

//Instruction parts
`define OPCODE		15:13
`define LOGBIT		12
`define DEST		11:8
`define SRC			7:4
`define TSRC		3:0
`define IMMED		7:0

//opcode (top 3 bits of instruction)
`define OPjrbr		3'b000	
`define OPlisi		3'b001
`define OPcocl		3'b010	// includes nl, mi, no, and lo
`define OPorst		3'b011	
`define OPeonop		3'b100	
`define OPalad		3'b101	
`define OPmlan		3'b110	
`define OPdlsr		3'b111	

module lnspipe(clk, halt, reset);
	input clk, reset;
	output reg halt;
	
	wire s1halt, s2halt, s3halt;
	wire REGwrite, SHIFTsel;
	wire [1:0] PCsel, REGinsel;
	wire [2:0] ALUop
	
	wire [`WORD] s1instruct, s1i8, s2LUTr, s2S, s2T, DMEMout, ALUout; // word sized values passed between stages
	wire [`REGSEL] addrS, addrT, addrD; // register select values
	
	stage1 one(clk, s1instruct, addrS, addrT, addrD, s1i8, s2S, PCsel);
	stage2 two(clk, s2S, s2T, s2LUTr, addrS, addrT, addrD, s1i8, REGinsel, REGwrite, SHIFTsel);
	stage3 three(clk, DMEMout, ALUout, s2S, s2T, s2LUTr, ALUop, DMEMwe);
	
	control Oracle(clk, PCsel, REGinsel, SHIFTsel, REGwrtie, ALUop, DMEMwe, s1instruct);
	
		
endmodule


module stage1(clk, s1instruct, addrS, addrT, addrD, i8, s2S, PCsel);
	input clk;
	input [1:0] PCsel;
	input [`WORD] s2S;
	
	output [`REGSEL] addrS, addrT, addrD;
	output [`WORD] i8, s1instruct;
	
	wire [`WORD] PCout, PCin;
	
	assign i8 = {{{s1instruct[7]},8}, s1instruct[7:0]};
	
	always@ (PCout, PCsel) begin
		if(PCsel == 0) PCin = PCout + 1;
		else if(PCsel == 1) PCin = PCout + 1 + i8;
		else if(PCsel == 2) PCin = s2S;
		else PCin = 0;
	end
	
	lsreg PC(clk, PCout, PCin, 1'b1, 1'b0);
	mainMem IMEM(clk, s1instruct, PCout, 1'b0);
	
	
endmodule

module stage2(clk, s2S, s2T, s2LUTr, addrS, addrT, addrD, s1i8, ALUout, DMEMout, REGinsel, REGwrite, SHIFTsel, addnotsub);
	input clk, SHIFTse, addnotsub;
	input [`REGSEL] addrSin, addrTin, addrDin;
	input [`WORD] ALUout, DMEMout, i8in;
	input [1:0] REGinsel;
	input SHIFTsel, REGwrite;
	
	output [`WORD] s2S, s2T, LUTr;
	
	wire [`WORD] din, SHIFTr, i8;
	wire [`REGSEL] addrS, addrT, addrD;
	
	assign LUTaddr = addnotsub ? s2T-s2S: s2T - s2S + 0h03c3;
	
	regfile REGS(clk, s2S, s2T, din, we, addrS, addrT, addrD, reset);
	shifters SHIFT(SHIFTr, s2S, s2T, SHIFTsel, i8);
	logLUT table(clk, LUTr, LUTaddr);
	
	always@ (posedge clk) begin;
		addrS <= addrSin;
		addrT <= addrTin;
		addrD <= addrDin;
		i8 	  <=	i8in;
	
	always@ (REGinsel, SHIFTr, DMEMout, ALUout, i8) begin
		if(REGinsel == 0) din = ALUout;
		else if(REGinsel == 1) din = i8;
		else if(REGinsel == 2) din = DMEMout;
		else din = SHIFTr;
	end
	
	
	
endmodule

module stage3(clk, DMEMout, ALUout, s2S, s2T, s2LUTr, ALUop, DMEMwe, addnotsub, lals);
	input clk, DMEMwe, addnotsub, lals;
	input [`WORD] s2S, s2T, s2LUTr;
	input [2:0] ALUop;
	
	output [`WORD] ALUout, DMEMout;
	
	wire [`WORD]  LUTr, ALUbin;
	
	always@ (posedge clk) LUTr <= s2LUTr;
	
	assign ALUbin = lals ? LUTr : s2T
	
	alu ALU(ALUout, s2S, ALUbin, ALUop);
	mainMem DMEM(clk, DMEMout, s2T, s2S, DMEMwe);
	
	
endmodule

module ls_reg(clk, out, in, c, reset);
	input clk, reset;                                            //input [`WORD] s2instruct, jrdest;
	input  c;                                             //input clk;
	input [`WORD] in;                                     //
	output reg [`WORD] out;                               //output [`WORD] s1instruct, sexi, i8;
	                         
	                                                      //output halt;
	always@ (posedge clk)begin                            //
		if(reset) out <= 0;
		else if (!c) out <= out;                               //wire [`WORD] brdest, pcin, pcout, instruct;
		else if(c) out <= in;                             //wire [`INSEL] pcsel;
		else out <= 16'bx;                                //wire controlhalt;
	end                                                   //
	                                                      //
endmodule       
	
module mainMem(clk, dataout, datain, addr, we);
	input clk, we;
	input [`WORD] datain, addr;
	output reg [`WORD] dataout;
	
	reg [`WORD] memory [`MEMSIZE];
	
//initial begin $readmemh("C:\Users\CJ\Documents\ee480_assignment_3\EE480_assignment3\EE480_assignment3.srcs\sources_1\new\test.vmem",memory); end
	// initial begin
	// memory[0] = 0;
	// memory[1] = 0;
	// memory[2] = 0;
	// memory[3] = 16'ha321; 
	// memory[4] = 16'h9000;
	// memory[5] = 16'h6321;
	// memory[6] = 16'h9000;
	// memory[7] = 16'hc321;
	// memory[8] = 16'h9000;
	// memory[9] = 16'h4321;
	// memory[10] = 16'h9000;
	// memory[11] = 16'h9000;
	// memory[12] = 16'h2339;
	// memory[13] = 16'h334f;
//	end
	always@ (addr, we) begin
		if (we) memory[addr] <= datain;
		dataout <= memory[addr];
	end
	
endmodule

module regfile(clk, outs, outt, din, we, sels, selt, seld, reset);
	input clk, we, reset;
	input [15:0] din; //write data if reg control is set to write
	input [3:0] sels, selt, seld; 
	
	output reg [15:0] outs, outt; 
	reg [15:0] r [15:0];
	
	always @(posedge clk) begin
		if(reset) begin 
			r[0] <= 16'h0;
			r[1] <= 16'h5;
			r[2] <= 16'h3;
			r[3] <= 16'h0;
		end
		else if(we) 	r[seld] <= din;
		else 			r[seld] = r[seld];	
	end
		
	
	always@ (sels, selt, we, reset)begin
		outs <= r[sels];
		outt <= r[selt];
	end
	//assign outs = r[sels];
	//assign outt = r[selt];
	
endmodule          


