module datapath(write_data,write_reg,RGWRITEFINAL,apc ,oldpc, clk) ;


    
input [31:0] oldpc;
input clk;
output [31:0] write_data ;
output [4:0]write_reg;
output RGWRITEFINAL;

wire [15:0] signextended;
wire [5:0] opcode;
wire [4:0] rs;
wire [4:0] rt;
wire [4:0] rd;
wire [15:0] signextendedout;
wire [5:0] opcodeout;
wire [4:0] rsout;
wire [4:0] rtout;
wire [4:0] rdout;
wire zf;
wire RegDst;

wire MemRead;
wire[1:0] load;
wire[1:0] eload;
wire[1:0] loadout;
wire[1:0] wloadout;
wire MemtoReg;
wire [3:0]ALUOp;
wire MemWrite;
wire AluSrc;
//wire RgWrite;
wire[31:0] memout;
wire[31:0] Aluresult;
wire[3:0] AluOut;
wire[31:0] Aluinput2;
wire [5:0] alufunc;
wire [4:0] result;
wire [31:0] signextend;
//wire [4:0] write_reg;
wire [31:0] read_data_1;
wire [31:0] read_data_2;
wire [31:0] extended;
wire [31:0] npc;
wire [31:0] instruction;
wire [31:0] shifted;
wire [4:0] RdRt;

wire eRgWrite;
wire eMemtoReg; 
wire eMemWrite;
wire  eAluSrc;
wire eBranch;
wire [3:0] eALUOp;
wire [4:0] eRdRt;
wire [31:0] eRead_data_1;
wire [31:0]eRead_data_2;
wire [31:0]eExtended ;
wire eMemread;
wire wregout,MemtoRegout, MemWriteout;
wire [4:0] RdRtout;
wire [31:0] qbout; 
wire regout, m2regout, wmemout;
wire [4:0] wRdRtout;
wire  [31:0] aluresultout, wqbout;
wire memregout, mem2regout;
wire [4:0] memRdRtout;
wire [31:0] memaluresultout;
output wire [31:0] apc;
wire [31:0] memdataout;
wire[31:0] wmemdataout;
wire rmemout;
wire[31:0] rpc;
wire Memreadout;
wire wmemregout, wmem2regout;
wire [4:0] wmemRdRtout;
wire [31:0] wmemaluresultout ;
wire [31:0]xpc;


wire [31:0]tpc;
wire [31:0]hpc;
wire pcsrc;
wire pcsrc2;
bigbitMux b (npc, xpc,pcsrc2,apc );

pc pc(tpc , clk  , oldpc );

IF IF( clk, tpc ,npc , signextended , opcode , rs, rt,  rd);

IFIDRegister IFIDRegister( signextendedout , opcodeout , rsout, rtout,  rdout , clk , signextended , opcode , rs, rt,  rd );

ID ID( signextendedout , opcodeout , rsout, rtout,  rdout , write_data, write_reg , clk, RGWRITEFINAL, MemtoReg, MemWrite, AluSrc,Branch,MemRead, load, ALUOp,RdRt,read_data_1, read_data_2, extended);

IDEXERegister IDEXERegister(RgWrite, MemtoReg,  MemRead , MemWrite, AluSrc,clk,Branch, load, ALUOp,RdRt,read_data_1, read_data_2, extended , eRgWrite, eMemtoReg, eMemread , eMemWrite, eAluSrc,eBranch,eload, eALUOp, eRdRt, eRead_data_1, eRead_data_2 ,eExtended);

EXE EXE(eRgWrite, eMemtoReg, eMemread , eMemWrite, eAluSrc ,eBranch, eload, eALUOp, eRdRt, eRead_data_1, eRead_data_2 ,eExtended, npc,hpc , Aluresult , pcsrc);

EXEMEMRegister EXEMEMRegister(clk, eRgWrite, eMemtoReg, eMemread , eMemWrite, eload,  eRdRt ,eRead_data_2 , Aluresult , regout, m2regout, rmemout, wmemout, wloadout , wRdRtout,aluresultout, wqbout);

MEM MEM(hpc , pcsrc, regout, m2regout, rmemout , wmemout ,  wloadout , wRdRtout,aluresultout, wqbout ,memdataout , xpc , pcsrc2 );

MemWbRegister MEMWBRegister(clk, regout, m2regout,wRdRtout,aluresultout, memdataout , wmemregout, wmem2regout,wmemRdRtout,wmemaluresultout, wmemdataout);

WB WB(RgWrite, MemtoReg,RdRt,Aluresult, wmemdataout, RGWRITEFINAL, write_reg, write_data);

  endmodule




module tb(); 
reg clk;
reg [31:0] pc;
 wire [31:0] apc;
    wire[31:0] write_data;
   wire [4:0]  RDRTFINALOUT;
   

//datapath d(npc,xpc,write_data,RDRTFINALOUT,RGWRITEFINAL, apc,pc , clk);
initial begin clk = 0;



forever begin #5 clk = ~clk; end
end



initial
begin
 pc <= 32'b00000000000000000000000000000000;

 $monitor("pc:%b apc:%b writedata:%b RDRTFINALOUT:%b RGWRITEFINAL:%b  ",  pc,apc , write_data,RDRTFINALOUT,RGWRITEFINAL ); 
#50 pc<= 32'b0000000000000000000000000000100;

// #100 pc<= 32'b0000000000000000000000000000100;
// #100 pc<= 32'b0000000000000000000000000001000;
// #100 pc<= 32'b0000000000000000000000000010000;
// #100 pc<= 32'b0000000000000000000000000010100;
// #100 pc<= 32'b0000000000000000000000000011000;
// #100 pc<= 32'b0000000000000000000000000011100;
// #100 pc<= 32'b0000000000000000000000000100000;
// #100 pc<= 32'b0000000000000000000000000100100;
// #100 pc<= 32'b0000000000000000000000000101000;
// #100 pc<= 32'b0000000000000000000000000101100;
// #100 pc<= 32'b0000000000000000000000000110000;



#500 $finish;
end

always @(apc)
begin
#5  pc<=apc;
end
 datapath d1(write_data,RDRTFINALOUT,RGWRITEFINAL, apc,pc , clk);

endmodule

//*************************************************************



module IF(
    input clk,
    input  [31:0] opc,
    output [31:0] npc,
    output  [15:0] signextended,
    output  [5:0] opcode,
    output  [4:0] rs, rt,  rd

    );

 

  adder pcadder(npc , opc );
  instructionmemory IM(signextended, opcode , rs ,rt , rd , opc );   

endmodule


//**************************************************************



module IFIDRegister(
 output reg [15:0] signextended,
    output reg [5:0] opcode,
    output reg [4:0] rs,
    output reg [4:0] rt,
    output reg [4:0] rd,
   
   input clk,
   input  [15:0] signextendedin,
   input  [5:0] opcodein,
   input [4:0] rsin,
   input  [4:0] rtin,
   input  [4:0] rdin

   
   
    );

  reg [31:0] mem;
    
    always @ (posedge clk)
    begin
    mem[31:26]=opcodein;
    mem[15:0]=signextendedin;
    mem[25:21]=rsin;
    mem[20:16]=rtin;
    mem[15:11]=rdin;
      
    end

    always @ (negedge clk)
    begin
      opcode = mem[31:26];
      signextended = mem[15:0];
      rs = mem[25:21];
      rt = mem[20:16];
      rd = mem[15:11];
    end
    
endmodule



//**************************************************************


module ID(
    input  [15:0] signextendedin,
    input  [5:0] opcodein,
    input [4:0] rsin,
    input  [4:0] rtin,
    input  [4:0] rdin,
    input [31:0] write_data,
    input [4:0] write_reg ,
    input clk,
    output RgWrite, MemtoReg, MemWrite, AluSrc,Branch,
    output MemRead,
    output[1:0] load,
    output[3:0]ALUOp,
    output [4:0] RdRt,
    output [31:0] read_data_1, read_data_2,
    output [31:0] extended
    );
wire RegDst;
controller ControlUnit(RegDst,Branch,MemRead,MemtoReg,MemWrite,AluSrc,RgWrite, ALUOp,load , opcodein,signextendedin[5:0]);
fivebitMux RegMux(rtin,rdin, RegDst, RdRt);
SignExtender SignExtender(signextendedin, extended);
rf RegFile(rsin, rtin,write_reg, write_data, RgWrite, read_data_1, read_data_2);

endmodule



//**************************************************************



module IDEXERegister(
    input RgWrite, MemtoReg,  Memread , input MemWrite,  AluSrc, clk,branch,
    input [1:0]load,
    input [3:0] ALUOp,
    input [4:0] RdRt,
    input [31:0] read_data_1, read_data_2, extended,
    output reg eRgWrite, eMemtoReg, output reg eMemread, output reg eMemWrite, eAluSrc,eBranch,
    output reg [1:0] eload,
    output reg [3:0] eALUOp,
    output reg [4:0] eRdRt,
    output reg [31:0] eRead_data_1, eRead_data_2 ,eExtended
    );
   reg wRgWrite, wMemtoReg,  wMemread , wMemWrite,  wAluSrc, wbranch;
   reg [1:0]wload;
   reg  [3:0] wALUOp;
   reg[4:0] wRdRt;
   reg  [31:0] wread_data_1, wread_data_2, wextended;

always @ (posedge clk)
begin
  wload <=load;
  wbranch  <=branch;
  wRgWrite <= RgWrite;
  wMemtoReg <=  MemtoReg;
  wMemread <= Memread;
  wMemWrite <= MemWrite;
  wAluSrc <=  AluSrc;
  wALUOp <= ALUOp;
  wread_data_1 <=read_data_1;
  wread_data_2 <= read_data_2;
  wextended <= extended;
  wRdRt <= RdRt;
end

always @ (negedge clk)
begin
  eload <= wload;
  eBranch <= wbranch;
  eRgWrite <= wRgWrite;
  eMemtoReg <= wMemtoReg;
  eMemread <= wMemread;
  eMemWrite <= wMemWrite;
  eAluSrc <= wAluSrc;
  eALUOp <= wALUOp;
  eRead_data_1 <= wread_data_1;
  eRead_data_2 <= wread_data_2;
  eExtended <= wextended;
  eRdRt <= wRdRt;
end

endmodule



//**************************************************************




module EXE(
  input eRgWrite, eMemtoReg, input eMemread, input eMemWrite, AluSrc , eBranch,
  input [1:0] load,
  input [3:0] AluOut, 
  input [4:0] RdRtin,
  input [31:0] read_data_1, read_data_2, extended, 
  input [31:0]pc, 
  output [31:0] npc,
  output[31:0] Aluresult,
  output pcsrc
	
	);
  wire[31:0] Aluinput2;
  wire [3:0] AlucOutput;
always@(AluSrc)
begin
$display("Aluresult: %b", AluSrc);
 end

 bigbitMux secondmux (read_data_2, extended , AluSrc , Aluinput2);
 
 alucontrol  alucont( AlucOutput, extended[5:0] , AluOut );
 newpc nnpc(npc, pcsrc, eBranch, zf ,pc , extended );
 ALU alu(Aluresult , zf , read_data_1 ,  Aluinput2 , AlucOutput );

endmodule



//**************************************************************




module EXEMEMRegister(
  input clk, wregin, m2regin, input rmemin , input wmemin,
  input [1:0] load,
  input [4:0] RdRtin,
  input [31:0] qbin, aluresultin,
  output reg wregout, m2regout, output reg rmemout , output reg wmemout,
  output reg [1:0] loadout,
  output reg [4:0] RdRtout,
  output reg [31:0] aluresultout, qbout
  );
   reg wwregout, wm2regout, wrmemout ,wwmemout;
   reg [1:0] wloadout;
  reg [4:0] wRdRtout;
  reg [31:0] waluresultout, wqbout;

always @ (posedge clk) begin
  wloadout <=load;
  wrmemout <= rmemin;
  wwregout <= wregin;
  wm2regout <= m2regin;
  wwmemout <= wmemin;
  wRdRtout <= RdRtin;
  waluresultout <= aluresultin;
  wqbout <= qbin;
end

always @ (negedge clk) begin
  loadout <= wloadout;
  rmemout <= wrmemout;
  wregout <=  wwregout;
  m2regout <= wm2regout;
  wmemout <= wwmemout;
  RdRtout <= RdRtout;
  aluresultout <= waluresultout;
  qbout <=  wqbout;
end
endmodule




//*********************************************MEM**************************************************


module MEM(
	input [31:0] hpc,
  input pcsrc , wregin, m2regin, input rmemin, input wmemin,
  input [1:0] load,
	input [4:0] RdRtin,
	input [31:0] aluresult, qb,

	output [31:0]  dataout , tpc,
  output pcsrc2
	);


assign tpc = hpc;
assign pcsrc2=pcsrc;
dataMemory DM(dataout, wmemin,rmemin,load , aluresult,qb );

endmodule

//**************************************Mem WB Register*******************************************



module MemWbRegister(
	input clk, wregin, m2regin,
	input [4:0] RdRtin,
	input [31:0] aluresultin, memdatain,
	output reg wregout, m2regout,
	output reg [4:0] RdRtout,
	output reg [31:0] aluresultout, memdataout
	);
  reg wwregout, wm2regout;
  reg [4:0] wRdRtout;
  reg [31:0] waluresultout, wmemdataout;

always @ (posedge clk) begin
	wwregout <= wregin;
	wm2regout <= m2regin;
	wRdRtout <= RdRtin;
	waluresultout <= aluresultin;
	wmemdataout <= memdatain;
end

always @ (negedge clk) begin
  wregout <= wwregout;
  m2regout <= wm2regout;
  RdRtout <= wRdRtout;
  aluresultout <= waluresultout;
  memdataout <= wmemdataout;
end

endmodule



//**************************************************************


module WB(
	input wregin, MemtoReg,
	input [4:0] RdRtin,
	input [31:0] AlucOutput, memdata,
	output wregout,
	output [4:0] RdRtout,
	output [31:0] write_data
	);

assign wregout = wregin;
assign RdRtout = RdRtin;

bigbitMux bigbitmux(AlucOutput, memdata , MemtoReg, write_data);
always@(write_data)
begin
$display("The value:%b is written in reg:%b" ,  write_data , RdRtin);
end
endmodule



//**************************************************************




//************************************************PC*******************************************************//
module pc(output reg [31:0]pc , input clk  , input [31:0] oldpc);

always @(posedge clk)
begin
pc = oldpc;
  end


endmodule


//************************************************Adder*******************************************************//
module adder(output[31:0]oldpc , input[31:0] pc);

assign oldpc=pc+4;
endmodule





//************************************************newPC*******************************************************//
module newpc (output [31:0]pc ,  output pcsrc, input branch ,  aluzero , input [31:0]oldpc , signextended);

assign pc = (branch == 1 && aluzero==1) ?  oldpc+(signextended*4):oldpc;
 
assign pcsrc = (branch == 1 && aluzero==1) ? 1:0;
endmodule



//**************************************instruction memory********************************//



module instructionmemory (output reg[15:0] signextended, output reg[5:0] opcode , output reg [4:0] rs , output reg [4:0] rt , output reg [4:0] rd , input[31:0] pc);
reg [31:0] data;
reg [31:0] registers[127:0];




always@(pc)
begin
data<=(pc==0)? 32'b0 : registers[pc];



// 10001100010000110000000000000000; //lw
// 10101100010000110000000000000000; //sw
// 10000100010000110000000000000000; //lh
// 10010100010000110000000000000000; //lhu
// 00000000010000110000000000100000; //add/sub
// 00000000010000110000000000100100;//and/or
// 00110000010000110000000000000010; //andi
// 00100000010000110000000000000011; //addi
// 00000000010000110000000010000000; //sll
// 00000000010000010000000001000010; //srl
// 10001100010000110000000000000000; //slt
// 00000000010000110000000000101010; //sltu






registers[0]  <= 32'b00000000000000000000000000000000;  
registers[4]  <= 32'b10101100010000110000000000000000; //sw i saved value:3 in address 2
registers[8]  <= 32'b10001100010000110000000000000000; //lw i loaded value of addres 2 in register 3
registers[12] <= 32'b10000100010000110000000000000000; //lh i loaded value of address 2 in register 3
registers[16] <= 32'b10010100010000110000000000000000; //lhu i loaded value of address 2 in register 3
registers[20] <= 32'b00000000101000110001100000100000; //add/sub added 5+3 and put in RD=3(0011)
registers[24] <= 32'b00000000110000110001100000100100; //and/or anded 6&4 and put in RD=(0011)
registers[28] <= 32'b00110000010000110000000000000010; //andi anded 2& 2(emidiate value) and put in RD(0011)
registers[32] <= 32'b00100000111000110000000000000011; //addi added 7 & 3(emidiate value) and put in RD(0011)
registers[36] <= 32'b00000000100000110001100010000000; //sll shift left 4(RS) by 2(shifting func) put in(0011)
registers[40] <= 32'b00000000100000010001100001000010; //srl shift right 4(rs) by 1
registers[44] <= 32'b10001100011001010001100000000000; //slt set 3(rs) less than 5(rt)
registers[48] <= 32'b00000000100000110001100000101010; //sltu set 4(rs) lessthan 3 (rt)

//registers[12] <=32'b10001100010000110000000000000000;


signextended <= data[15:0];
opcode <= data[31:26];
rs<=data[25:21];
rt<=data[20:16];
rd<=data[15:11];

end


endmodule



//***********************************Register file*******************************************//

module rf(read_reg_1, read_reg_2, write_reg, write_data, regWrite, read_data_1, read_data_2); 

input [4:0] read_reg_1, read_reg_2, write_reg;
input [31:0] write_data;
input regWrite;
output reg [31:0] read_data_1, read_data_2;
reg [31:0] registers[31:0];

always @ (read_reg_1, read_reg_2) begin

registers[0] <= 32'b00000000000000000000000000000000; 
registers[1] <= 32'b00000000000000000000000000000001; 
registers[2] <= 32'b00000000000000000000000000000010;  
registers[3] <= 32'b00000000000000000000000000000011; 
registers[4] <= 32'b00000000000000000000000000000100; 
registers[5] <= 32'b00000000000000000000000000000101; 
registers[6] <= 32'b00000000000000000000000000000110; 
registers[7] <= 32'b00000000000000000000000000000111; 


read_data_1 <= (read_reg_1==0)? 32'b0 : registers[read_reg_1];
read_data_2 <= (read_reg_2==0)? 32'b0 : registers[read_reg_2]; end
initial
begin
if(regWrite) begin
registers[write_reg] <= write_data; end
end 
endmodule 

//*********************************mux*********************************************//

module bigbitMux (
  input [31:0] a, b,
  input select,
  output [31:0] out
  );

  assign out = (select == 0) ?  a : b;

endmodule

//*********************************mux*********************************************//

module fivebitMux(
  input [4:0] a, b,
  input select,
  output [4:0] out
  );

  assign out = (select == 0) ?  a : b;

endmodule


//*********************************Controller*****************************************//

module controller(output reg RegDst,Branch, MemRead, MemtoReg,MemWrite,AluSrc,RgWrite,output reg [3:0] ALUOp , output reg[1:0] load,input [5:0] instruction,func);

always @(instruction)
begin

case(instruction)                    //Addi
  6'b001000: begin
RegDst <= 0;
Branch <= 0; 
MemRead <= 0 ;
MemtoReg <= 0;
ALUOp <= 4'b0011;
MemWrite <= 0;
AluSrc <= 1;
RgWrite <= 1;
load<=2'b00;
end

6'b001100: begin //andi
RegDst <= 0;Branch <= 0; MemRead <= 0 ;MemtoReg <= 0;ALUOp <= 4'b0100;
MemWrite <= 0;AluSrc <= 1;RgWrite <= 1; load<=2'b00;
end

6'b001101: begin //ori
RegDst <= 0;Branch <= 0; MemRead <= 0 ;MemtoReg <= 0;ALUOp <= 4'b0101;
MemWrite <= 0;AluSrc <= 1;RgWrite <= 1; load<=2'b00;
end

6'b100011: begin //lw
RegDst <= 0;Branch <= 0; MemRead <= 1 ;MemtoReg <= 1;ALUOp <= 4'b0000; 
MemWrite <= 0;AluSrc <= 1;RgWrite <= 1; load<=2'b00;
end

6'b101011: begin 
RegDst <= 0;Branch <= 0; MemRead <= 0 ;MemtoReg <= 0;ALUOp <= 4'b0000;MemWrite <= 1;AluSrc <= 1;RgWrite <= 0;  load<=2'b00; //sw
end

6'b000100: begin //beq
RegDst <= 0; Branch <= 1; MemRead <= 0 ;MemtoReg <= 0;ALUOp <= 4'b0001;MemWrite <= 0;AluSrc <= 0;RgWrite <= 0; load<=2'b00;
end

6'b100101: begin //lhu
RegDst <= 0;Branch <= 0; MemRead <= 1; MemtoReg <= 1;ALUOp <= 4'b0000;MemWrite <= 0;AluSrc <= 1;RgWrite <= 1; load<=2'b10;
end

6'b100001: begin //lh
RegDst <= 0;Branch <= 0; MemRead <= 1 ;MemtoReg <= 1;ALUOp <= 4'b0000;MemWrite <= 0;AluSrc <= 1;RgWrite <= 1; load<=2'b01;
end

default: 
begin //R-type
if(func==0 || func==2)
begin
RegDst <= 1;Branch <= 0; MemRead <= 0 ;MemtoReg <= 0;ALUOp <= 4'b0010 ;MemWrite <= 0;AluSrc <= 1;RgWrite <= 1; load<=2'b00;
end
else
begin
RegDst <= 1;Branch <= 0; MemRead <= 0 ;MemtoReg <= 0;ALUOp <= 4'b0010 ;MemWrite <= 0;AluSrc <= 0;RgWrite <= 1; load<=2'b00;
end
end
endcase
end
endmodule

//********************************************Sign extend********************************************************//

module SignExtender( input [15:0] in,  output reg [31:0] out );
    
    
    
   always@(in) 
    begin
    if(in[15]==0)
        begin
            out = { {16'b0000000000000000} , {in} };
        end
    else
        begin
            out = { {16'b1111111111111111} , {in} };
        end
    end
    
endmodule 


//**********************************Alu control***********************************************************************//

module alucontrol( AlucOutput, Alufunction , ALUOp );
input [5:0] Alufunction;
input [3:0] ALUOp;

output reg [3:0] AlucOutput;
always@(Alufunction, ALUOp)
begin
case(ALUOp)
4'b0100 : AlucOutput <= 4'b0000; //Andi
4'b0101 : AlucOutput <= 4'b0001; //Ori
4'b0011 : AlucOutput <= 4'b0010; //Addi
4'b0000 : AlucOutput <= 4'b0010; //lw and sw
4'b0001 : AlucOutput <= 4'b0110; //beq
4'b0010: begin
case(Alufunction)   //R-type
6'b100000: AlucOutput <= 4'b0010; //add
6'b100010: AlucOutput <= 4'b0110; //sub
6'b100100: AlucOutput <= 4'b0000; //and
6'b100101: AlucOutput <= 4'b0001; //or
6'b101010: AlucOutput <= 4'b0111; //slt
6'b101011: AlucOutput <= 4'b1011; //sltu
6'b000000: AlucOutput <= 4'b0011; //sll
6'b000010: AlucOutput <= 4'b0100; //srl

                                //lh
                                //lhu
endcase
end
endcase
end
endmodule




//**********************************new alu************************************************************************//

module ALU (OUT, ZeroFlag, In1, In2, ALUfunct);
input [31:0] In1, In2; 
input [3:0] ALUfunct; 
reg my_int;
wire fbit , sbit;
output reg [31:0] OUT; 
output reg ZeroFlag;
assign fbit=In1[0];
assign sbit=In2[0];

always @ (In1, In2, ALUfunct) 
begin
    my_int = 1'd0;
    if 
        (In1 == In2) ZeroFlag = 1;
    else
        ZeroFlag = 0;
end

always @ (In1, In2, ALUfunct) begin
case (ALUfunct)
4'b0010 : OUT = In1 + In2; 
4'b0110 : OUT = In1 - In2; 
4'b0000 : OUT = In1 & In2;  
4'b0001 : OUT = In1 | In2; 
4'b0111 : if(In1<In2) 
            begin
            OUT= 32'd1;
              end
          else
            begin 
          OUT= 32'd0; 
          end
//4'b1100 :   OUT = ~(In1|In2);
4'b1011 :   if(sbit>fbit)
          begin
          
          OUT= 32'd1; 
          end
      else
         if(sbit<fbit)
          begin
          OUT= 32'd0; 
         end
      else
        if(fbit==0 )
          begin
          if(In1<In2) 
      begin
      OUT= 32'd1;
          end
          else
        begin 
      OUT= 32'd0; 
      end
      end
  else
     if(In1<In2) 
        begin
        OUT= 32'd0;
          end
          else
        begin 
      OUT= 32'd1; 
      end

 4'b0011  : 
  
      case(In2)
      31'b0   : OUT <= In1;  
      31'b0000000000000000000000010000000  : OUT <= In1 << 2;  
      31'b0000000000000000000000100000000  : OUT <= In1 << 3;  
      31'b0000000000000000000001100000000  : OUT <= In1 << 4;  
      31'b0000000000000000000010000000000  : OUT <= In1 << 5;  
      31'b0000000000000000000010100000000  : OUT <= In1 << 6;  
      31'b0000000000000000000011000000000  : OUT <= In1 << 7;  
      31'b0000000000000000000011100000000  : OUT <= In1 << 8;  
      31'b0000000000000000000100000000000  : OUT <= In1 << 9;  
    endcase

4'b0100  : 
  
      case(In2)
      31'b0   : OUT <= In1;  
      31'b0000000000000000000000001000010   : OUT <= In1 >> 1; 
      31'b0000000000000000000000010000010   : OUT <= In1 >> 2;  
      31'b0000000000000000000000011000010   : OUT <= In1 >> 3;  
      31'b0000000000000000000000100000010   : OUT <= In1 >> 4;  
      31'b0000000000000000000000101000010   : OUT <= In1 >> 5;  
      31'b0000000000000000000000110000010   : OUT <= In1 >> 6;  
      31'b0000000000000000000001110000010   : OUT <= In1 >> 7;  
      31'b0000000000000000000010000000010   : OUT <= In1 >> 8;  
   
    endcase

endcase 
end
endmodule




//**********************************data memory***********************************************************************//
module dataMemory(out,write,read,load ,address,data);
input clk;
input read;
input write;
input [1:0] load;
input [31:0] address,data;
output reg[31:0] out;


reg [31:0] ram[1023:0];
integer i;
wire [15:0] extend;
wire [31:0] lh;
wire [31:0] backup;

assign extend=15'b0;



SignExtender signextender(ram[address][15:0] , lh);

assign backup[15:0] =ram[address][15:0];
assign backup[31:16] =extend;

initial begin 


for(i=0; i<1023 ; i=i+1)
begin
ram[i]=0;
end
ram[1] =32'b00000000000000001000000000000001;
end

//assign out = ram[address];

 
 


 
//  always@(ram[address])
// begin
// $display("ram[address] %b", ram[address]);
//  end
 



always@(*)
begin
case(read)
0: if(write) begin ram[address] <= data; end
1: case(load)
    2'b01:begin  out<=lh; end
    2'b10: begin   out[15:0] <= ram[address][15:0];
                   out[31:16] <=extend;
            end 
    default: out <= ram[address];
    endcase
endcase
end


 always@(write)
 begin
 $display("The value:%b is srored in memory position:%b",data , ram[address]);
  end



endmodule


 //******************************ALU ADDER********************************************************//