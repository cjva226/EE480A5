`define WORD		15:0
`define HALFWORD	7:0
`define REGSEL		3:0

`define REGSIZE		15:0
`define	MEMSIZE		1023:0

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

// 

module lnspipe(clk, halt, reset);
	input clk, reset;
	output halt;
	
	//wire s1halt, s2halt, s3halt;
	wire REGwe, SHIFTsel, addnotsub, lals, condlatch, DMEMwe, logsig, s3addr, lisi;
	wire [1:0] PCsel, REGinsel;
	wire [2:0] ALUop;
	
	wire [`WORD] s1instruct, s2instruct, s3instruct, s1i8, LUTaddr, s2S, s2T, DMEMout, ALUout, PCout; // word sized values passed between stages
	wire [`HALFWORD] condition;
	wire [`REGSEL] addrS, addrT, addrD/*, s1addrS, s1addrT, s1addrD*/; // register select values
	
	//assign addrS = s3addr ? s3instruct[`SRC] : s1addrS;
	//assign addrT = s3addr ? s3instruct[`TSRC] : s1addrT;
	//assign addrD = s3addr ? s3instruct[`DEST] : s1addrD;
	//always@ (posedge clk) $display(" %d --- addrD = %h", $time, addrD);
	
	stage1 stageone(clk, PCout, s1instruct, addrS, addrT, addrD, s1i8, s2S, PCsel, reset);//(clk, s1instruct, addrS, addrT, addrD, i8, s2S, PCsel)
	stage2 stagetwo(clk, s2S, s2T, LUTaddr, condition, addrS,   addrT,   addrD,   s1i8, ALUout, DMEMout, REGinsel, REGwe, SHIFTsel, lisi, addnotsub, condlatch, s3addr, reset);
	//		 	   (clk, s2S, s2T, LUTaddr, condition, addrSin, addrTin, addrDin, i8in, ALUout, DMEMout, REGinsel, REGwe, SHIFTsel, addnotsub, condlatch, s3addr, reset
	stage3 stagethree(clk, DMEMout, ALUout, s2S, s2T, LUTaddr, ALUop, DMEMwe, addnotsub, lals, logsig);
	//				 (clk, DMEMout, ALUout, s2S, s2T, LUTaddr, ALUop, DMEMwe, addnotsub, lals, logsig)
	
	control Oracle(clk, halt, s3instruct, s2instruct,  s3addr, PCsel, REGinsel, SHIFTsel, REGwe, ALUop, DMEMwe, lals, lisi, logsig, condition, condlatch, s1instruct);
	

endmodule

module control(clk, halt, s3instructO, s2instructO, s3addr, PCsel, REGinsel, SHIFTsel, REGwe, ALUop, DMEMwe, lals, lisi, logsig, condition, condlatch, s1instruct);
	input clk;
	input [`WORD] s1instruct;
	input [`HALFWORD] condition;
	
	output [`WORD]	s3instructO, s2instructO;
	output [2:0] 	ALUop;
	output [1:0] 	PCsel, REGinsel;
	output			SHIFTsel, REGwe, DMEMwe, logsig, lals, lisi, condlatch, s3addr, halt;
	
	wire	[2:0]	ALUop1, ALUop2;
	wire	[1:0]	PCsel2, PCsel3;
	wire 	[1:0]	REGinsel1, REGinsel3;
	wire			SHIFTsel1, SHIFTsel3, REGwe1, REGwe3, DMEMwe1, DMEMwe2, logsig1, logsig2, lals1, lals2, s3addr1, s3addr3, halt2, halt3, condelatch1, condlatch3, lisi1, lisi3;
	reg [`WORD] s2instruct, s3instruct; // look here for clock skew
	
	assign s3instructO = s3instruct;
	assign s2instructO = s2instruct;
	
	always@ (posedge clk) begin
		$display("%d --- s1instruct = %h -- s2instruct = %h -- s3instruct = %h", $time, s1instruct, s2instruct, s3instruct);
		// $display("%d --- condlatch = %b", $time, condlatch); 
		s2instruct <= s1instruct;
		s3instruct <= s2instruct;
	end
	
	control_logic decoder1(halt, PCsel,  REGinsel1, SHIFTsel1, REGwe1, ALUop1, DMEMwe1, lals1, lisi,  logsig1, condlatch1, s3addr, s1instruct, s2instruct, condition);
	control_logic decoder2(halt2, PCsel2, REGinsel,  SHIFTsel,  REGwe,  ALUop2, DMEMwe2, lals2, lisi1, logsig2, condlatch, s3addr1, s2instruct, s3instruct, condition);
	control_logic decoder3(halt3, PCsel3, REGinsel3, SHIFTsel3, REGwe3, ALUop,  DMEMwe,  lals, lisi3,  logsig,  condlatch3, s3addr3, s3instruct, 16'b0, 	  condition);
	
endmodule

module control_logic(halt, PCsel, REGinsel, SHIFTsel, REGwe, ALUop, DMEMwe, lals, lisi, logsig, condlatch, s3addr, instruct, previnstruct, condition);
	input [`WORD] instruct;
	input [`HALFWORD] condition;
	
	wire [2:0] cond;
	
	input [`WORD] previnstruct;
	output reg [2:0] ALUop;
	output reg [1:0] PCsel, REGinsel;
	output reg SHIFTsel, REGwe, DMEMwe, lals, lisi, logsig, condlatch, s3addr, halt;

	assign cond = instruct[11:9];
        	

//	always@ (posedge clk)begin
//		previnstruct <= instruct;
//	end

	initial halt <= 0;
	always@ (instruct, condition, previnstruct, cond) begin
		PCsel <= 0; REGinsel <= 0; SHIFTsel <= 0; REGwe <= 0; ALUop <= 0; DMEMwe <= 0; logsig <= 0; condlatch <= 0; s3addr <= 0; lals <= 0; lisi <= 0; halt <= 0;
		if(instruct == 0) halt <= 1;
		else begin
		case(instruct[`OPCODE])
			`OPjrbr	:	 begin
							//$display(PCsel);
							//if(instruct == 0) halt <= 1;//sy
							if(instruct[8] == 0) 
								begin
									
									if(condition[cond]) 	PCsel <= 1; // br
									else PCsel <= 0; // fall through
									
								end
							else begin
								if(condition[cond] == 1) begin	PCsel <= 2; /*$display(PCsel);*/ end // jr
								else PCsel <= 0;//sy
							end
						end
			`OPlisi	: 	begin
							if(instruct[12] == 1) 	begin 	PCsel <= 0; REGinsel <= 3; SHIFTsel <= 0; REGwe <= 1; lisi <= 1; end //si
							else				begin	PCsel <= 0; REGinsel <= 1; REGwe 		<= 1;	end //li
						end
			`OPcocl	: 
						begin
							if(instruct[`DEST] == 0) begin 		PCsel <= 0; condlatch <= 1; end// cocl
							else if(instruct[`TSRC] == 0 && instruct[12] == 1) begin PCsel <= 0; ALUop <= 7; end //nl
							else if(instruct[`TSRC] == 0 && instruct[12] == 0) begin PCsel <= 0; ALUop <= 5; end //mi
							else if(instruct[`TSRC] == 1) begin PCsel <= 0; ALUop <= 3; end //no
							else if(instruct[`TSRC] == 2) begin PCsel <= 0; end //lo
						end
			`OPorst	: 	begin
							if(instruct[`DEST] == 0) begin 		PCsel <= 0; DMEMwe <= 1; end //st
							else begin 							PCsel <= 0; ALUop <= 2; end //or
						end
			`OPalad	: 	begin
							if(instruct[12] == 0) begin PCsel <= 0; ALUop <= 0;  end //ad
							else begin 					PCsel <= 0; ALUop <= 0; lals <= 1; logsig <= 1; end	//al
						end
			`OPmlan	:	begin
							if(instruct[12] == 0) begin PCsel <= 0; ALUop <= 1; end //an
							else begin 					PCsel <= 0; ALUop <= 0; logsig <= 1; end //ml
						end
			`OPdlsr	: 	begin
							if(instruct[12] == 0) begin	PCsel <= 0; REGinsel <= 3; SHIFTsel <= 1; REGwe <= 1; end //sr
							else begin PCsel <= 0; ALUop <= 6; logsig <= 1; end //dl
						end
			`OPeonop	:	begin
							if(instruct[12] == 0) begin end //eo
							else //nop
							begin
								case(previnstruct[`OPCODE])
									`OPjrbr	: PCsel <= 0;
									`OPlisi	: begin PCsel <= 0; s3addr <= 1; end
									`OPcocl	: 
												begin
													PCsel <= 0;
													if(previnstruct[`DEST] != 0)// clco
														begin
														REGwe <= 1; s3addr <= 1;
															if(previnstruct[`TSRC] == 2)  REGinsel <= 2;
															else REGinsel <= 0;
														end
												end
									`OPorst	:	
												begin
													if(previnstruct[`DEST] != 0) begin REGwe <= 1; REGinsel <= 0; s3addr <= 1; end
												end
									`OPeonop	:	begin
													PCsel <= 0;
													if(previnstruct[12] == 0) begin REGwe <= 1; REGinsel <= 0; s3addr <= 1;end
												end
									`OPalad	:	begin PCsel <= 0; REGwe <= 1; REGinsel <= 0; s3addr <= 1;end
									`OPmlan	:	begin PCsel <= 0; REGwe <= 1; REGinsel <= 0; s3addr <= 1;end
									`OPdlsr	:	begin PCsel <= 0;
													if(previnstruct[12] == 1)
													begin
														REGwe <= 1; REGinsel <= 0; s3addr <= 1;
													end
												end
									default:	PCsel <= 0;
								endcase
							end
						end
					0: begin halt <= 1; end
			default : PCsel <= 3;
		endcase
		end
	end
	
	
	
endmodule

module stage1(clk, PCout, s1instruct, addrS, addrT, addrD, i8, s2S, PCsel, reset);
	input clk;
	input reset;
	input [1:0] PCsel;
	input [`WORD] s2S;
	
	output [`REGSEL] addrS, addrT, addrD;
	output [`WORD] i8, s1instruct;
	
	reg [`WORD]  PCin;
	output [`WORD] PCout;
	
	assign    i8 = {{8{s1instruct[7]}}, s1instruct[7:0]};
	assign addrS = s1instruct[`SRC];
	assign addrT = s1instruct[`TSRC];
	assign addrD = s1instruct[`DEST];
	
	//always @(posedge clk) $display("%d === s1addrD = %h", $time, addrD);
	
	//always@(posedge reset) if(reset) PCin = 1;
	
	always@ (PCout, PCsel, i8, s2S, posedge reset) begin
		$display("%d --- %d", $time, s2S);
		if(reset) PCin = 1;
		else if	(PCsel == 0) PCin = PCout + 1;
		else if(PCsel == 1) PCin = PCout + 1 + i8;
		else if(PCsel == 2) PCin = s2S;
		else if(PCsel == 3) PCin = 0;
		else PCin = 0;
	end
	
	lsreg PC(clk, PCout, PCin, 1'b1, reset);
	mainMem IMEM(clk, s1instruct, 1'b0, PCout, 1'b0);
	
	
endmodule

module stage2(clk, s2S, s2T, LUTaddr, condition, addrSin, addrTin, addrDin, i8in, ALUout, DMEMout, REGinsel, REGwe, SHIFTsel, lisi,addnotsub, condlatch, s3addr, reset);
	input clk;
	input condlatch, reset, s3addr, lisi;
	input [`REGSEL] addrSin, addrTin, addrDin;
	input [`WORD] ALUout, DMEMout, i8in;
	input [1:0] REGinsel;
	input SHIFTsel, REGwe;
	
	output [`WORD] s2S, s2T, LUTaddr;
	output reg [`HALFWORD] condition;
	output addnotsub;
	
	wire [`WORD] SHIFTr;
	wire[`HALFWORD] SHIFTrbttm;
	reg [`REGSEL] addrS, addrT, addrD;
	reg [`WORD] din, LUTaddr, i8;
	
	//assign LUTaddr	 	= s2T-s2S; 
	always@ (s2S, s2T) begin
		if (s2T-s2S > 16'h03c3) LUTaddr <= 16'h03C3;
		else LUTaddr <= s2T-s2S;
	end
	assign SHIFTrbttm = {s2S[7:0], i8[7:0]}; 
	assign SHIFTr		= SHIFTsel ? s2S >> s2T : SHIFTrbttm;
	assign addnotsub 	= s2S[15];
	
	regfile REGS(clk, s2S, s2T, din, REGwe, addrS, addrT, addrD, reset);
	//			 clk, outs, outt, din, we, sels, selt, seld, reset
	//shifters SHIFT(SHIFTr, s2S, s2T, SHIFTsel, i8);
	//logLUTad tab(clk, LUTr, LUTaddr);
	
	
	always@ (posedge clk, posedge reset) begin
		//$display(reset);
		$display("%d  %h  %b  %b %b %b %b %b", $time, din, SHIFTr, REGinsel, addrD, addrS, addrT, s2S);
		if(reset == 1) condition <= 8'b00000001;
		else if(condlatch) begin    
				if 			(s2S>s2T) 	condition <= 8'b00001111; // f lt le eq ne ge gt t
				else if 	(s2S<s2T) 	condition <= 8'b01101001;                            
				else if		(s2S == s2T)condition <= 8'b00010001; 
				else 					condition <= 8'bzzzzzzzz;
		end	
	end
	
	
	always@ (posedge clk) begin
		// if(reset == 1) condition <= 8'b00000001;
		// else if(condlatch) begin    
				// if 			(s2S>s2T) 	condition <= 8'b00001111; // f lt le eq ne ge gt t
				// else if 	(s2T>s2S) 	condition <= 8'b01101001;                            
				// else if		(s2S == s2T)condition <= 8'b00010001; 
				// else 					condition <= 8'bzzzzzzzz;
		// end	
		//$display("time: %d --- signal = %d", $time, s3addr);
		if(s3addr == 0 && lisi == 0)begin
			addrS <= addrSin;
			addrT <= addrTin;
			addrD <= addrDin;
			end
		else if(s3addr == 0 && lisi == 1)begin
			addrS <= addrDin;
			addrT <= addrTin;
			addrD <= addrDin;
		end
		else if(s3addr == 1)
		begin
			addrS <= addrS;
			addrT <= addrT;
			addrD <= addrD;
		end
	
		i8 	  <=	i8in;
		
		//$display("%d --- din = %h -- REGinsel = %h -- REGwe = %h", $time, din, REGinsel, REGwe);
	end
	
	always@ (REGinsel, SHIFTr, DMEMout, ALUout, i8, reset) begin
		if(reset) din <= 0;
		else if(REGinsel == 0) 		din <= ALUout;
		else if(REGinsel == 1) 	din <= i8;
		else if(REGinsel == 2) 	din <= DMEMout;
		else din <= SHIFTr;
	end
	
	// always@ (s2S, s2T)begin // what is this for?
		
	// end
	
endmodule

module stage3(clk, DMEMout, ALUout, s2S, s2T, LUTaddr, ALUop, DMEMwe, addnotsub, lals, logsig);
	input clk, DMEMwe, addnotsub, lals, logsig;
	input [`WORD] s2S, s2T, LUTaddr;
	input [2:0] ALUop;
	
	output [`WORD] ALUout, DMEMout;
	
	wire [`WORD]  ALUbin;
	wire [`WORD] LUTra, LUTrs;
	
	
	//always@ (posedge clk) LUTr <= s2LUTr;
	
	
	assign ALUbin = lals ? (addnotsub ? LUTra: LUTrs) : s2T;
	assign LUTra = 0;
	assign LUTrs = 0;
	//logLUTad tab(clk, LUTra, LUTaddr);
	//logLUTsu tab2(clk, LUTrs, LUTaddr);
	alu ALU(ALUout, s2S, ALUbin, ALUop, logsig, lals);
	mainMem DMEM(clk, DMEMout, s2T, s2S, DMEMwe);
	
	
endmodule

module lsreg(clk, out, in, c, reset);
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
	

	 initial begin
	 memory[0] = 16'h0000;
	 $readmemh0(memory);
	 
	end
	always@ (addr, we, datain, memory[addr]) begin
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
	
	initial begin $readmemh1(r); end
	
	//forever r[0] = 0;
	//assign r[0] = 0;
	
	always @(posedge clk, we, seld, sels, selt, r[sels], r[selt]) begin
		r[0] <= 0;
		if(we) 	r[seld] <= din;
		else 	r[seld] = r[seld];	
		
		// outs <= r[sels];
		// outt <= r[selt];
	end
	
	always@(posedge r[sels], posedge r[selt])
		begin
			outs <= r[sels];
			outt <= r[selt];
		end
		
	
	// always@ (sels, selt, we, reset, r[sels], r[selt])begin
		// outs <= r[sels];
		// outt <= r[selt];
	// end
	//assign outs = r[sels];
	//assign outt = r[selt];
	
endmodule          

module alu(out, a, b, c, logsig, lals);                                     
	input [`WORD] a, b;                                                
	input [2:0] c;
	input logsig, lals;                                           
	output reg [`WORD] out;                                                                                     

	always@ (a, b, c, logsig, lals) begin                                                
		if(logsig == 1)
		begin
			case(c)
				0:	begin
						if(lals == 1)// al
						begin
							if(a == 16'h8000 || b == 16'h8000) out = 16'h8000; // consoder tickdefines
							else if(a == 16'hffff || a == 16'h7fff) out = {a[15] & 16'hffff};
							else if(b == 16'hffff || b == 16'h7fff) out = {b[15] & 16'hffff};
							else if(a == 0)	out = b; else if(b == 0) out = a;
							else 									out = a + b;
						end
						else//ml
						begin
							if(a == 16'h8000 || b == 16'h8000) out = 16'h8000;
							else if(a == 16'hffff || a == 16'h7fff) out = {a[15] & 16'hffff};
							else if(b == 16'hffff || b == 16'h7fff) out = {b[15] & 16'hffff};
							else if(a == 0 || b == 0) 				out = 0;
							else 									out = a + b;
						end
					end
						
					
				6: begin out = {!a[15], a[14:0]}; end//nl
				7:	begin //dl
						if(a == 16'h8000 || b == 16'h8000) out = 16'h8000;
						else if(a == 16'hffff || a == 16'h7fff) out = {a[15] & 16'hffff};
						else if(b == 16'hffff || b == 16'h7fff) out = {b[15] & 16'hffff};
						else if(b == 0)				out = 16'h8000;
						else if(a == 0)				out = 0;
						else 									out = a - b;
					end
				default: out = 16'bx;
			endcase
		end
		else
		begin
		case(c)                                                      
			0: out = a + b;   			
			1: out = a & b;                                         
			2: out = a | b;                                          
			3: out = ~a;                                             
			4: out = a ^ b;
			5: out = -a;
			7: out = a - b;
		endcase  
			
		end
	end                                                            
endmodule

module LUTad(clk, LUTr, LUTaddr);
	input clk;
	input [`WORD] LUTaddr;
	output reg [`WORD] LUTr;
	
	reg [`WORD] mem [1023:0];
	
	initial begin
		$readmemh2(mem);
	end
	
	always@ (posedge clk) begin
		LUTr <= mem[LUTaddr];
	end
endmodule

module LUTsu(clk, LUTr, LUTaddr);
	input clk;
	input [`WORD] LUTaddr;
	output reg [`WORD] LUTr;
	
	reg [`WORD] mem [1023:0];
	
	initial begin
		$readmemh3(mem);
	end
	
	always@ (posedge clk) begin
		LUTr <= mem[LUTaddr];
	end
endmodule


	
	


module pipetb;
	reg clk = 0;
    reg reset = 0;
	wire halt;
    integer i = 0;
	
	lnspipe uut(clk, halt, reset);
	
	initial begin
		$dumpfile;
		$dumpvars(1, uut);
			#50 reset = 1;
			#50 reset = 0;
			while(!halt && i < 1024) begin
				#10 clk = 1;
				#10 clk = 0;
				i = i+1;
			end
		$finish;
		
	end
	
	
endmodule
