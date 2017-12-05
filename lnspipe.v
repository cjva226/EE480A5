`timescale 1ns/1ps
`define WORD		15:0
`define HALFWORD	7:0
`define REGSEL		3:0

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

// to do
// finish cotrol logic
// implement alu
// add nl and negate log
// add special cases to alu

module lnspipe(clk, halt, reset);
	input clk, reset;
	output reg halt;
	
	//wire s1halt, s2halt, s3halt;
	wire REGwe, SHIFTsel, addnotsub;
	wire [1:0] PCsel, REGinsel;
	wire [2:0] ALUop;
	
	wire [`WORD] s1instruct, s1i8, s2LUTr, s2S, s2T, DMEMout, ALUout; // word sized values passed between stages
	wire [`REGSEL] addrS, addrT, addrD; // register select values
	
	stage1 one(clk, s1instruct, addrS, addrT, addrD, s1i8, s2S, PCsel);
	stage2 two(clk, s2S, s2T, s2LUTr, condition, addrS, addrT, addrD, s1i8, REGinsel, REGwe, SHIFTsel, addnotsub, condlatch);
	stage3 three(clk, DMEMout, ALUout, s2S, s2T, s2LUTr, ALUop, DMEMwe);
	
	control Oracle(clk, PCsel, REGinsel, SHIFTsel, REGwe, ALUop, DMEMwe, logsig, addnotsub, condlatch, s1instruct);
	

endmodule

module control(clk, PCsel, REGinsel, SHIFTsel, REGwe, ALUop, DMEMwe, logsig, addnotsub, condlatch, s1instruct);
	input clk;
	input [`WORD] s1instruct;
	
	output [2:0] 	ALUop;
	output [1:0] 	PCsel, REGinsel;
	output			SHIFTsel, REGwe, DMEMwe, logsig, addnotsub, condlatch;
	
	wire	[2:0]	ALUop1, ALUop2;
	wire	[1:0]	PCsel2, PCsel3;
	wire 	[1:0]	REGinsel1, REGinsel3;
	wire			SHIFTsel1, SHIFTsel3, REGwe1, REGwe3, DMEMwe1, DMEMwe2, logsig1, logsig2, addnotsub1, addnotsub3;
	
	reg s2instruct, s3instruct; // look here for clock skew
	
	always@ (posedge clk) begin
		s2instruct <= s1instruct;
		s3instruct <= s2instruct;
	end
	
	control_logic decoder1(PCsel,  REGinsel1, SHIFTsel1, REGwe1, ALUop1, DMEMwe1,  logsig1, addnotsub1, condlatch, s1instruct, condition);
	control_logic decoder2(PCsel2, REGinsel,  SHIFTsel,  REGwe,  ALUop2, DMEMwe2, logsig2, addnotsub,  condlatch, s2instruct, condition);
	control_logic decoder3(PCsel3, REGinsel3, SHIFTsel3, REGwe3, ALUop,  DMEMwe,  logsig,  addnotsub3, condlatch, s3instruct, condition);
	
endmodule

module control_logic(PCsel, REGinsel, SHIFTsel, REGwe, ALUop, DMEMwe, logsig, addnotsub, condlatch, instruct, condition);
	input [`WORD] instruct;
	input [`HALFWORD] condition;
	
	reg [`WORD] previnstruct;
	output reg [2:0] ALUop;
	output reg [1:0] PCsel, REGinsel;
	output reg SHIFTsel, REGwe, DMEMwe, logsig, addnotsub, condlatch;
	
	always@ (posedge instruct)begin
		previnstruct <= instruct;
	end
	
	always@ (instruct) begin
		PCsel <= 0; REGinsel <= 0; SHIFTsel <= 0; REGwe <= 0; ALUop <= 0; DMEMwe <= 0; logsig <= 0; addnotsub <= 0; condlatch <= 0;
		case(instruct[`OPCODE])
			`OPjrbr	:	 begin
							if(instruct[8] == 1) 
							begin
								if(condition[instruct[11:9]] == 1) 	PCsel <= 1; // br
								else PCsel <= 0; // fall through
							end
							else if(condition[instruct[11:9]] == 1) 	PCsel <= 2; // jr
							else 								PCsel <= 0;	// fall through
						end
			`OPlisi	: 	begin
							if(instruct[12]==1) 	begin 	PCsel <= 0; REGinsel <= 3; SHIFTsel 	<= 0; end //si
							else				begin	PCsel <= 0; REGinsel <= 1; REGwe 		<= 1;	end //li
						end
			`OPcocl	: 
						begin
							if(instruct[`DEST] == 0) begin 		PCsel <= 0; condlatch <= 1; end// cocl
							else if(instruct[`TSRC] == 0 & instruct[12] == 1) begin PCsel <= 0; ALUop <= 7; end //nl
							else if(instruct[`TSRC] == 0 & instruct[12] == 0) begin PCsel <= 0; ALUop <= 5; end //mi
							else if(instruct[`TSRC] == 1) begin PCsel <= 0; ALUop <= 3; end //no
							else if(instruct[`TSRC] == 2) begin PCsel <= 0; end //lo
						end
			`OPorst	: 	begin
							if(instruct[`DEST] == 0) begin 		PCsel <= 0; DMEMwe <= 1; end //st
							else begin 							PCsel <= 0; ALUop <= 2; end //or
						end
			`OPalad	: 	begin
							if(instruct[12] == 0) begin PCsel <= 0; ALUop <= 0;  end //ad
							else begin 					PCsel <= 0; ALUop <= 0; end	//al
						end
			`OPmlan	:	begin
							if(instruct[12] == 0) begin PCsel <= 0; ALUop <= 1; end //an
							else begin 					PCsel <= 0; ALUop <= 0; end //ml
						end
			`OPdlsr	: 	begin
							if(instruct[12] == 0) begin	PCsel <= 0; REGinsel <= 3; SHIFTsel <= 1; REGwe <= 1; end //sr
							else begin PCsel <= 0; ALUop <= 6; end //dl
						end
			`OPeonop	:	begin
							if(instruct[12] == 0) begin end //eo
							else //nop
							begin
								case(previnstruct)
									`OPjrbr	: PCsel <= 0;
									`OPlisi	: PCsel <= 0;
									`OPcocl	: 
												begin
													PCsel <= 0;
													if(previnstruct[`DEST] != 0)// clco
														begin
														REGwe <= 1;
															if(previnstruct[`TSRC] == 2)  REGinsel <= 2;
															else REGinsel <= 0;
														end
												end
									`OPorst	:	
												begin
													if(previnstruct[`DEST] != 0) begin REGwe <= 1; REGinsel <= 0; end
												end
									`OPeonop	:	begin
													PCsel <= 0;
													if(previnstruct[12] == 0) begin REGwe <= 1; REGinsel <= 0; end
												end
									`OPalad	:	begin PCsel <= 0; REGwe <= 1; REGinsel <= 0; end
									`OPmlan	:	begin PCsel <= 0; REGwe <= 1; REGinsel <= 0; end
									`OPdlsr	:	begin PCsel <= 0;
													if(previnstruct[12] == 1)
													begin
														REGwe <= 1; REGinsel <= 0;
													end
												end
									default:	PCsel <= 3;
								endcase
							end
						end
			default : PCsel <= 3;
		endcase
	end
	
	
	
endmodule

module stage1(clk, s1instruct, addrS, addrT, addrD, i8, s2S, PCsel);
	input clk;
	input [1:0] PCsel;
	input [`WORD] s2S;
	
	output [`REGSEL] addrS, addrT, addrD;
	output [`WORD] i8, s1instruct;
	
	reg [`WORD] PCout, PCin;
	
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

module stage2(clk, s2S, s2T, s2LUTr, condition, addrSin, addrTin, addrDin, i8in, ALUout, DMEMout, REGinsel, REGwe, SHIFTsel, addnotsub, condlatch);
	input clk, addnotsub, condlatch;
	input [`REGSEL] addrSin, addrTin, addrDin;
	input [`WORD] ALUout, DMEMout, i8in;
	input [1:0] REGinsel;
	input SHIFTsel, REGwe;
	
	output [`WORD] s2S, s2T, s2LUTr;
	output reg [`HALFWORD] condition;
	
	wire [`WORD] SHIFTr, LUTaddr;
	reg [`REGSEL] addrS, addrT, addrD, i8, din;
	
	assign LUTaddr = addnotsub ? s2T-s2S: s2T - s2S + 0'h03c3; // may need to change
	//assign SHIFTr	= SHIFTsel ? 
	
	regfile REGS(clk, s2S, s2T, din, REGwe, addrS, addrT, addrD, 1'b0 );
	shifters SHIFT(SHIFTr, s2S, s2T, SHIFTsel, i8);
	logLUT tab(clk, LUTr, LUTaddr);
	
	always@ (posedge clk) begin
		addrS <= addrSin;
		addrT <= addrTin;
		addrD <= addrDin;
		i8 	  <=	i8in;
	end
	always@ (REGinsel, SHIFTr, DMEMout, ALUout, i8) begin
		if(REGinsel == 0) 		din = ALUout;
		else if(REGinsel == 1) 	din = i8;
		else if(REGinsel == 2) 	din = DMEMout;
		else din = SHIFTr;
	end
	
	always@ (s2S, s2T)begin
		
	end
	
	always@(posedge clk) begin
		if(condlatch) begin    
			if 			(s2S>s2T) 	condition <= 8'b00001111; // f lt le eq ne ge gt t
			else if 	(s2T>s2S) 	condition <= 8'b01101001;                            
			else					condition <= 8'b00010001;                                    
		end	
	end
	
endmodule

module stage3(clk, DMEMout, ALUout, s2S, s2T, s2LUTr, ALUop, DMEMwe, addnotsub, lals);
	input clk, DMEMwe, addnotsub, lals;
	input [`WORD] s2S, s2T, s2LUTr;
	input [2:0] ALUop;
	
	output [`WORD] ALUout, DMEMout;
	
	wire [`WORD]  ALUbin;
	reg	[`WORD] LUTr;
	
	always@ (posedge clk) LUTr <= s2LUTr;
	
	
	assign ALUbin = lals ? LUTr : s2T;
	
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

module alu(out, a, b, c);                                     
	input [`WORD] a, b;                                                
	input [2:0] c;                                                   
	output reg [`WORD] out;                                                                                     

	always@ (a, b, c) begin                                                
		case(c)                                                      
			0: out = a + b;   			
			1: out = a & b;                                         
			2: out = a | b;                                          
			3: out = ~a;                                             
			4: out = a ^ b;
			5: out = -a;
			
			
		endcase                                                      
	end                                                            
endmodule

	


