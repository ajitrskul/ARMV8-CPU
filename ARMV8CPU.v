`timescale 1ns / 1ps
`define OPCODE_ANDREG 11'b?0001010???
`define OPCODE_ORRREG 11'b?0101010???
`define OPCODE_ADDREG 11'b?0?01011???
`define OPCODE_SUBREG 11'b?1?01011???

`define OPCODE_ADDIMM 11'b?0?10001???
`define OPCODE_SUBIMM 11'b?1?10001???

`define OPCODE_MOVZ   11'b110100101??

`define OPCODE_B      11'b?00101?????
`define OPCODE_CBZ    11'b?011010????

`define OPCODE_LDUR   11'b??111000010
`define OPCODE_STUR   11'b??111000000

`define SIZE 1024

module singlecycle(
  input 	     resetl,
  input [63:0]      startpc,
  output reg [63:0] currentpc,
  output [63:0]     MemtoRegOut,  // this should be
  // attached to the
  // output of the
  // MemtoReg Mux
  input 	     CLK
);

  // Next PC connections
  wire [63:0] 			     nextpc;       // The next PC, to be updated on clock cycle

  // Instruction Memory connections
  wire [31:0] 			     instruction;  // The current instruction

  // Parts of instruction
  wire [4:0] 			     rd;            // The destination register
  wire [4:0] 			     rm;            // Operand 1
  wire [4:0] 			     rn;            // Operand 2
  wire [10:0] 			     opcode;

  // Control wires
  wire 			     reg2loc;
  wire 			     alusrc;
  wire 			     mem2reg;
  wire 			     regwrite;
  wire 			     memread;
  wire 			     memwrite;
  wire 			     branch;
  wire 			     uncond_branch;
  wire [3:0] 			     aluctrl;
  wire [2:0] 			     signop;

  // Register file connections
  wire [63:0] 			     regoutA;     // Output A
  wire [63:0] 			     regoutB;     // Output B

  // ALU connections
  wire [63:0] 			     aluout;
  wire 			     	   zero;
  wire [63:0]				  aluin;

  // Sign Extender connections
  wire [63:0] 			     extimm;

  //Data Memory connection
  wire [63:0] 				 readdata;

  // PC update logic
  always @(negedge CLK)
    begin
      if (resetl)
        currentpc <= #3 nextpc;
      else
        currentpc <= #3 startpc;
    end

  // Parts of instruction
  assign rd = instruction[4:0];
  assign rm = instruction[9:5];
  assign rn = reg2loc ? instruction[4:0] : instruction[20:16];
  assign opcode = instruction[31:21];

  InstructionMemory imem(
    .Data(instruction),
    .Address(currentpc)
  );

  //connections
  control control(
    .reg2loc(reg2loc),
    .alusrc(alusrc),
    .mem2reg(mem2reg),
    .regwrite(regwrite),
    .memread(memread),
    .memwrite(memwrite),
    .branch(branch),
    .uncond_branch(uncond_branch),
    .aluop(aluctrl),
    .signop(signop),
    .opcode(opcode)
  );
	
  Mux1 Mux1(
    .out(aluin),
    .input1(regoutB),
    .input2(extimm),
    .Ctrl(alusrc)
  );
//registerfile connections
  RegisterFile RegisterFile(
    .BusA(regoutA),
    .BusB(regoutB),
    .BusW(MemtoRegOut),
    .RA(instruction[9:5]),
    .RB(rn), 
    .RW(rd),
    .RegWr(regwrite),
    .Clk(CLK)
  );
//nextPC logic connections
  NextPClogic NextPClogic(
    .NextPC(nextpc),
    .CurrentPC(currentpc),
    .SignExtImm64(extimm),
    .Branch(branch),
    .ALUZero(zero),
    .Uncondbranch(uncond_branch)
  );


//ALU connections
  ALU ALU(
    .BusW(aluout),
    .Zero(zero),
    .BusA(regoutA),
    .BusB(aluin), 
    .ALUCtrl(aluctrl)
  );
  //signExtender connections
  SignExtender SignExtender(
    .BusImm(extimm),
    .Imm32(instruction[25:0]),
    .Ctrl(signop)
  );

  Mux1 Mux2 (
    .out(MemtoRegOut),
    .input1(aluout),
    .input2(readdata),
    .Ctrl(mem2reg)
  );
	//data memory connection
  DataMemory DataMemory(
    .ReadData(readdata),
    .Address(aluout),
    .WriteData(regoutB) ,
    .MemoryRead(memread) ,
    .MemoryWrite(memwrite) ,
    .Clock(CLK)
  );


endmodule

module SignExtender(BusImm, Imm32, Ctrl); 
  //inputs and outputs
  output reg [63:0] BusImm; 
  input [25:0] Imm32; 
  input [2:0] Ctrl; 

  always @(*)
    begin
      //$display("Imm32);
      case(Ctrl) //Ctrl tells which option it is
        3'b000://Checks the ctrl bit for I-type
          begin
            BusImm = {52'b0,Imm32[21:10]}; //sign extends immediate
          end
        3'b001: //checks the ctrl bit for B-type
          begin
            BusImm = {{36{Imm32[25]}},Imm32[25:0], 2'b0}; //extends BR address
          end
        3'b010: //checks the first bit for CB
          begin
            BusImm = {{43{Imm32[23]}},Imm32[23:5], 2'b0};//BR_address
          end
        3'b011: //checks the first bit for D-type
          begin
            BusImm = {{55{Imm32[20]}},Imm32[20:12]}; //sign extends DT_ address
          end
       3'b100: //MOVZ
          begin
            BusImm = {48'b0, Imm32[20:5]}; //sign extend the input
            case(Imm32[21:20])
                 2'b00:
                 begin
                   BusImm = BusImm << 32; //shift 32 to the left
                 end
                 2'b01:
                 begin
                   BusImm = BusImm; //dont shift at all
                 end
                 2'b10:
                 begin
                   BusImm = BusImm << 48; //shift 48 bits to the left
                 end
                 2'b11:
                 begin
                   BusImm = BusImm << 16; //shift 16 bits to the left
                 end
              default:
                BusImm <= BusImm; //default case
            endcase
          end
        default:
          BusImm = 64'bX;
      endcase
    end
endmodule

module ALU(BusW, Zero , BusA , BusB , ALUCtrl ) ;
  //start inputs and outputs
  output reg [63:0] BusW;
  output Zero;
  input [63:0] BusA, BusB;
  input [3:0] ALUCtrl;
  assign Zero = (BusW==64'b0)?1'b1:1'b0;
  always @(*)
    begin
      case (ALUCtrl) //each CTRL value has a different operation

        4'b0000:
          begin
            BusW = BusA & BusB; //AND
          end
        4'b0001:
          begin
            BusW = BusA | BusB; //OR
          end
        4'b0010:
          begin
            BusW = BusA + BusB; //ADD
          end
        4'b0110:
          begin
            BusW = BusA - BusB; // SUB
          end
        4'b0111:
          begin
            BusW = BusB; // PassB
          end
        default:
          BusW =64'bx;
      endcase
    end
endmodule

module RegisterFile(BusA, BusB, BusW, RA, RB, RW, RegWr, Clk);
  output [63:0] BusA;
  output [63:0] BusB;
  input [63:0] BusW;
  input [4:0] RA;
  input [4:0] RB;
  input [4:0] RW;
  input RegWr;
  input Clk;
  reg [63:0] registers [31:0];
  initial registers[31] = 64'b0;
  assign #2 BusA = (RA!=5'd31)?registers[RA]:64'b0; //set output to 31 if RA accessing register 31, else return element in specified register
  assign #2 BusB = (RB!=5'd31)?registers[RB]:64'b0; //set output of BusB if RB is accessing register 31, else return element at specified register

  always @ (negedge Clk) begin
    //$display("BusB:%h", BusB);
    if(RegWr)
      registers[RW] <= #3 BusW;
  end
endmodule


//`timescale 1ns / 1ps
module control(
  output reg 	reg2loc,
  output reg 	alusrc,
  output reg 	mem2reg,
  output reg 	regwrite,
  output reg 	memread,
  output reg 	memwrite,
  output reg 	branch,
  output reg 	uncond_branch,
  output reg [3:0] aluop,
  output reg [2:0] signop,
  input [10:0] 	opcode
);

  always @(*)
    begin
      casez (opcode)
        `OPCODE_ANDREG: //control bits for and
          begin
            reg2loc       = 1'b0;
            alusrc        = 1'b0;
            mem2reg       = 1'b0;
            regwrite      = 1'b1;
            memread       = 1'b0;
            memwrite      = 1'b0;
            branch        = 1'b0;
            uncond_branch = 1'b0;
            aluop         = 4'b0000;
            signop        = 3'bxxx;
          end
        `OPCODE_ORRREG: //control bits for or
          begin
            reg2loc       = 1'b0;
            alusrc        = 1'b0;
            mem2reg       = 1'b0;
            regwrite      = 1'b1;
            memread       = 1'b0;
            memwrite      = 1'b0;
            branch        = 1'b0;
            uncond_branch = 1'b0;
            aluop         = 4'b0001;
            signop        = 3'bxxx;
          end
        `OPCODE_ADDREG: //control bits for regular ADD
          begin
            reg2loc       = 1'b0;
            alusrc        = 1'b0;
            mem2reg       = 1'b0;
            regwrite      = 1'b1;
            memread       = 1'b0;
            memwrite      = 1'b0;
            branch        = 1'b0;
            uncond_branch = 1'b0;
            aluop         = 4'b0010;
            signop        = 3'bxxx;
          end
        `OPCODE_SUBREG: //control bits for regular SUB
          begin
            reg2loc       = 1'b0;
            alusrc        = 1'b0;
            mem2reg       = 1'b0;
            regwrite      = 1'b1;
            memread       = 1'b0;
            memwrite      = 1'b0;
            branch        = 1'b0;
            uncond_branch = 1'b0;
            aluop         = 4'b0110;
            signop        = 3'bxxx;
          end
        `OPCODE_ADDIMM: //control bits for ADD
          begin
            reg2loc       = 1'b0;
            alusrc        = 1'b1;
            mem2reg       = 1'b0;
            regwrite      = 1'b1;
            memread       = 1'b0;
            memwrite      = 1'b0;
            branch        = 1'b0;
            uncond_branch = 1'b0;
            aluop         = 4'b0010;
            signop        = 3'b000;
          end
        `OPCODE_SUBIMM: //control bits for sub
          begin
            reg2loc       = 1'b0;
            alusrc        = 1'b1;
            mem2reg       = 1'b0;
            regwrite      = 1'b1;
            memread       = 1'b0;
            memwrite      = 1'b0;
            branch        = 1'b0;
            uncond_branch = 1'b0;
            aluop         = 4'b0110;
            signop        = 3'b000;
          end
        `OPCODE_MOVZ: //control bits for MOVZ
          begin
            reg2loc       = 1'bX;
            alusrc        = 1'b1;
            mem2reg       = 1'b0;
            regwrite      = 1'b1;
            memread       = 1'b0;
            memwrite      = 1'b0;
            branch        = 1'b0;
            uncond_branch = 1'b0;
            aluop         = 4'b0111;
            signop        = 3'b100;
          end
        `OPCODE_B: //control bits for Branch
          begin
            reg2loc       = 1'bX;
            alusrc        = 1'bX;
            mem2reg       = 1'bX;
            regwrite      = 1'b0;
            memread       = 1'b0;
            memwrite      = 1'b0;
            branch        = 1'bX;
            uncond_branch = 1'b1;
            aluop         = 4'bXXXX;
            signop        = 3'b001;
          end
        `OPCODE_CBZ: //contrl bits for CBZ
          begin
            reg2loc       = 1'b1;
            alusrc        = 1'b0;
            mem2reg       = 1'bX;
            regwrite      = 1'b0;
            memread       = 1'b0;
            memwrite      = 1'b0;
            branch        = 1'b1;
            uncond_branch = 1'b0;
            aluop         = 4'b0111;
            signop        = 3'b010;
          end
        `OPCODE_LDUR: //control bits for load
          begin
            reg2loc       = 1'bX;
            alusrc        = 1'b1;
            mem2reg       = 1'b1;
            regwrite      = 1'b1;
            memread       = 1'b1;
            memwrite      = 1'b0;
            branch        = 1'b0;
            uncond_branch = 1'b0;
            aluop         = 4'b0010;
            signop        = 3'b011;
          end
        `OPCODE_STUR: //control bits for store command
          begin
            reg2loc       = 1'b1;
            alusrc        = 1'b1;
            mem2reg       = 1'bX;
            regwrite      = 1'b0;
            memread       = 1'b0;
            memwrite      = 1'b1;
            branch        = 1'b0;
            uncond_branch = 1'b0;
            aluop         = 4'b0010;
            signop        = 3'b011;
          end
        default:
          begin
            reg2loc       = 1'bx;
            alusrc        = 1'bx;
            mem2reg       = 1'bx;
            regwrite      = 1'bx;
            memread       = 1'bx;
            memwrite      = 1'bx;
            branch        = 1'bx;
            uncond_branch = 1'bx;
            aluop         = 4'bxxxx;
            signop        = 3'bxxx;
          end
      endcase
    end

endmodule

module InstructionMemory(Data, Address);
   parameter T_rd = 20;
   parameter MemSize = 40;
   
   output [31:0] Data;
   input [63:0]  Address;
   reg [31:0] 	 Data;
   
   /*
    * ECEN 350 Processor Test Functions
    * Texas A&M University
    */
   
   always @ (Address) begin

      case(Address)

	/* Test Program 1:
	 * Program loads constants from the data memory. Uses these constants to test
	 * the following instructions: LDUR, ORR, AND, CBZ, ADD, SUB, STUR and B.
	 * 
	 * Assembly code for test:
	 * 
	 * 0: LDUR X9, [XZR, 0x0]    //Load 1 into x9
	 * 4: LDUR X10, [XZR, 0x8]   //Load a into x10
	 * 8: LDUR X11, [XZR, 0x10]  //Load 5 into x11
	 * C: LDUR X12, [XZR, 0x18]  //Load big constant into x12
	 * 10: LDUR X13, [XZR, 0x20]  //load a 0 into X13
	 * 
	 * 14: ORR X10, X10, X11  //Create mask of 0xf
	 * 18: AND X12, X12, X10  //Mask off low order bits of big constant
	 * 
	 * loop:
	 * 1C: CBZ X12, end  //while X12 is not 0
	 * 20: ADD X13, X13, X9  //Increment counter in X13
	 * 24: SUB X12, X12, X9  //Decrement remainder of big constant in X12
	 * 28: B loop  //Repeat till X12 is 0
	 * 2C: STUR X13, [XZR, 0x20]  //store back the counter value into the memory location 0x20
	 */
	

        //given test data that performs the assembly code above
        63'h000: Data = 32'hF84003E9;
        63'h004: Data = 32'hF84083EA;
        63'h008: Data = 32'hF84103EB;
        63'h00c: Data = 32'hF84183EC;
        63'h010: Data = 32'hF84203ED;
        63'h014: Data = 32'hAA0B014A;
        63'h018: Data = 32'h8A0A018C;
        63'h01c: Data = 32'hB400008C;
        63'h020: Data = 32'h8B0901AD;
        
        63'h024: Data = 32'hCB09018C; 
        63'h028: Data = 32'h17FFFFFD;
        63'h02c: Data = 32'hF80203ED; //STUR 1111 1000 0000 0010 0000 0011 1110 1101
        63'h030: Data = 32'hF84203ED;  //One last load to place stored value on memdbus for test checking.
        
        
        //start of our test code
        63'h034: Data = 32'hD2e24689; //Movz x9 1234 lsl 48
        63'h038: Data = 32'hD2CACF01; //Movz x1 5678 lsl 32
        63'h03c: Data = 32'h8B010129; //ADD X9, X9, X1
        63'h040: Data = 32'hD2B35781; //Movz x1 9abc lsl 16
        63'h044: Data = 32'h8B010129; //ADD x9, X9, x1
        63'h048: Data = 32'hD29BDE01; //Movz x1 def0 lsl 0
        63'h04c: Data = 32'h8B010129; //ADD x9, X9, X1
        63'h050: Data = 32'hF80283E9; //STUR x9, [xzr, 0x28]
        63'h054: Data = 32'hF84283EA; //LDUR x10, [xzr, 0x28]
        

	
	default: Data = 32'hXXXXXXXX;
      endcase
   end
endmodule

module NextPClogic(NextPC, CurrentPC, SignExtImm64, Branch, ALUZero, Uncondbranch); 
  input [63:0] CurrentPC, SignExtImm64; 
  input Branch, ALUZero, Uncondbranch; 
  output reg [63:0] NextPC; 
  reg [63:0] TempIMM;
  always@(*)
    begin
      TempIMM = SignExtImm64;
      if (Uncondbranch == 1'b1) //instruction is unconditional branch
        begin
          NextPC <= CurrentPC+TempIMM;
        end
      else if (Branch == 1'b1) //current instruction is conditional branch
        begin
          if (ALUZero == 1'b1)
            NextPC <= CurrentPC+TempIMM;
          else
            NextPC <= CurrentPC+4;
        end
      else //default nextPC logic
        begin
          NextPC <= CurrentPC+4;
        end

    end
endmodule

module DataMemory(ReadData , Address , WriteData , MemoryRead , MemoryWrite , Clock);
  input [63:0]      WriteData;
  input [63:0]      Address;
  input             Clock, MemoryRead, MemoryWrite;
  output reg [63:0] ReadData;

  reg [7:0] 	     memBank [`SIZE-1:0];


  // This task is used to write arbitrary data to the Data Memory by
  // the intialization block.
  task initset;
    input [63:0] addr;
    input [63:0] data;
    begin
      memBank[addr] =  data[63:56] ; // Big-endian for the win...
      memBank[addr+1] =  data[55:48];
      memBank[addr+2] =  data[47:40];
      memBank[addr+3] =  data[39:32];
      memBank[addr+4] =  data[31:24];
      memBank[addr+5] =  data[23:16];
      memBank[addr+6] =  data[15:8];
      memBank[addr+7] =  data[7:0];
    end
  endtask


  initial
    begin
      // preseting some data in the data memory used by test #1

      // Address 0x0 gets 0x1
      initset( 64'h0,  64'h1);  //Counter variable
      initset( 64'h8,  64'ha);  //Part of mask
      initset( 64'h10, 64'h5);  //Other part of mask
      initset( 64'h18, 64'h0ffbea7deadbeeff); //big constant
      initset( 64'h20, 64'h0); //clearing space

      // Add any data you need for your tests here.

    end

  // This always block reads the data memory and places a double word
  // on the ReadData bus.
  always @(posedge Clock)
    begin
      if(MemoryRead)
        begin
          ReadData[63:56] <=  memBank[Address];
          ReadData[55:48] <=  memBank[Address+1];
          ReadData[47:40] <=  memBank[Address+2];
          ReadData[39:32] <=  memBank[Address+3];
          ReadData[31:24] <=  memBank[Address+4];
          ReadData[23:16] <=  memBank[Address+5];
          ReadData[15:8] <=  memBank[Address+6];
          ReadData[7:0] <=  memBank[Address+7];
          //%display("DMEM %h",ReadData);
        end
    end

  // This always block takes data from the WriteData bus and writes
  // it into the DataMemory.
  always @(posedge Clock)
    begin
      if(MemoryWrite)
        begin
          memBank[Address] <= #3 WriteData[63:56] ;
          memBank[Address+1] <= #3 WriteData[55:48];
          memBank[Address+2] <= #3 WriteData[47:40];
          memBank[Address+3] <= #3 WriteData[39:32];
          memBank[Address+4] <= #3 WriteData[31:24];
          memBank[Address+5] <= #3 WriteData[23:16];
          memBank[Address+6] <= #3 WriteData[15:8];
          memBank[Address+7] <= #3 WriteData[7:0];
          // Could be useful for debugging:
          //$display("Writing Address:%h Data:%h",WriteData, memBank['h8]);
        end
    end
endmodule

module Mux1(out, input1, input2, Ctrl);
  output [63:0] out;
  input [63:0] input1, input2;
  input Ctrl;
  assign out = Ctrl ? input2 : input1;
endmodule