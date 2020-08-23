/*
 * This IP is the Lite RISC-V RV32I dual bus implementation.
 * 
 * Copyright (C) 2018  Iulian Gheorghiu (morgoth@devboard.tech)
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

`timescale 1ns / 1ps

/*
 * Interrupt and priority encoder.
 */
`include "core-io-h.v"
`include "def-h.v"
 
 module int_encoder_dual_bus # (
		parameter VECTOR_INT_TABLE_SIZE = 0
		)(
		input rst,
		input [((VECTOR_INT_TABLE_SIZE == 0) ? 0 : VECTOR_INT_TABLE_SIZE-1):0]int_sig_in,
		output int_request,
		output reg[((VECTOR_INT_TABLE_SIZE > 127) ? 7 :
					(VECTOR_INT_TABLE_SIZE > 63) ? 6 :
					(VECTOR_INT_TABLE_SIZE > 31) ? 5 :
					(VECTOR_INT_TABLE_SIZE > 15) ? 4 :
					(VECTOR_INT_TABLE_SIZE > 7) ? 3 :
					(VECTOR_INT_TABLE_SIZE > 3) ? 2 :
					(VECTOR_INT_TABLE_SIZE > 1) ? 1 : 0) : 0]int_vect
		);

integer j;
always @*
begin
	if(VECTOR_INT_TABLE_SIZE != 0)
	begin
		int_vect <= 0;
		for (j=VECTOR_INT_TABLE_SIZE-1; j>=0; j=j-1)
		if (int_sig_in[j]) 
			int_vect <= j+1;
	end
end

assign int_request = (int_vect != 0 && VECTOR_INT_TABLE_SIZE != 'h0);

endmodule
/*
 * !Interrupt and priority encoder.
 */

module risc_v_lite_dual_bus #
	(
	parameter PLATFORM = "XILINX",
	parameter ADDR_BUS_WIDTH = 32,
	parameter RESET_VECTOR = 32'h8000,
	parameter SYNCHRONOUS_ROM = "TRUE",
	parameter SYNCHRONOUS_RAM = "TRUE",
	parameter EXTENSION_C = "TRUE",
	parameter EXTENSION_M = "TRUE",
	parameter EXTENSION_MDIV = "FALSE",
	parameter WATCHDOG_CNT_WIDTH = 0,
	parameter VECTOR_INT_TABLE_SIZE = 0,
	parameter STALL_INPUT_LINES = 1,
	parameter NEG_CLK_REG_OUTS = "FALSE"
	)(
	input rst,
	input clk,
	input [STALL_INPUT_LINES - 1:0]core_stall,
	output [ADDR_BUS_WIDTH - 1:0]pgm_addr_out,
	input [31:0]pgm_inst_in,
	output reg [ADDR_BUS_WIDTH - 1:0]ram_addr_out,
	output reg [31:0]ram_out,
	output reg ram_wr_w,
	output reg ram_wr_h,
	output reg ram_wr_b,
	input [31:0]ram_in,
	input [(VECTOR_INT_TABLE_SIZE == 0 ? 0 : VECTOR_INT_TABLE_SIZE - 1):0]int_sig,
	output reg [(VECTOR_INT_TABLE_SIZE == 0 ? 0 : VECTOR_INT_TABLE_SIZE - 1):0]int_rst,
	output int_en
    );

reg [ADDR_BUS_WIDTH - 1:0]PC;
assign pgm_addr_out = PC;
reg [31:0]instruction;
wire [ADDR_BUS_WIDTH - 1:0]PC_FOR_JAL_JALR = (SYNCHRONOUS_ROM == "TRUE") ? PC : PC + ((EXTENSION_C == "TRUE" && SYNCHRONOUS_ROM == "FALSE") ? (pgm_inst_in[1:0] == 2'b11 ? 4 : 2) : 4);
reg [31:0]data_in_int;
wire [ADDR_BUS_WIDTH - 1:0]PC_FOR_AUIPC_AND_REL_JMP = (SYNCHRONOUS_ROM == "TRUE") ? PC - 4 : PC;
//reg CSR_UIE = 1'b1;

reg [ADDR_BUS_WIDTH - 1:0]RIP; /* Return Interrupt Pointer */

wire [4:0]rs1a = instruction[19:15];
wire [31:0]rs1;
wire [4:0]rs2a = instruction[24:20];
wire [31:0]rs2;
wire [4:0]rda = instruction[11:7];
wire [31:0]rd_alu;
reg [31:0]rd;
reg rdw;

reg data_write_sysreg_w;
reg data_write_sysreg_h;
reg data_write_sysreg_b;
//reg data_read_sysreg_w;
//reg data_read_sysreg_h;
//reg data_read_sysreg_b;


reg STAGE_2;
reg SKIP_EXECUTION;
reg ENTER_INTERRUPT;
reg DELAY_INT_REGISTERING;
reg core_inst_decode_fault;
wire arith_inst_decode_fault;
reg arith_inst_decode_fault_delayed_1c;
reg core_inst_decode_fault_no_c;
reg core_inst_decode_fault_delayed_no_c_1c;
reg core_stall_delayed_1c;
wire core_stall_int = |core_stall;
wire INSTRUCTION_FAULT = core_inst_decode_fault & arith_inst_decode_fault_delayed_1c & core_inst_decode_fault_delayed_no_c_1c & ~core_stall_int & ~core_stall_delayed_1c;

reg [7:0]reg_mstatus;					//Machine status register.
reg [7:0]reg_mie;						//Machine interrupt-enable register.
reg [ADDR_BUS_WIDTH - 1:0]reg_mscratch;	//Scratch register for machine trap handlers.
reg [ADDR_BUS_WIDTH - 1:0]reg_mepc;		//Machine exception program counter.
reg [7:0]reg_mcause;					//Machine trap cause.
reg [ADDR_BUS_WIDTH - 1:0]reg_mbadaddr;	//Machine bad address.

assign int_en = reg_mstatus[`REG_MSTATUS_MIE_bp];

reg [31:0]PERFORMANCE_CNT;

always @ (posedge clk)
begin
    if(rst)
    begin
		reg_mstatus <= 'h00;
        reg_mscratch <= RESET_VECTOR;
    end
    else
    begin
        if(data_write_sysreg_w)
        begin
            case(ram_addr_out[12:0])
            `REG_MSTATUS: reg_mstatus <= ram_out;
            `REG_MSCRATCH: reg_mscratch <= ram_out;
            endcase
        end
    end
end

reg [31:0]data_in_int_tmp;
reg [31:0]data_mem_in_int;

always @ *
begin
	case(ram_addr_out[12:0])
		`REG_MSTATUS: data_in_int_tmp = reg_mstatus;
		`REG_MSCRATCH: data_in_int_tmp = reg_mscratch;
		default: data_in_int_tmp = 32'h0;
	endcase
	data_mem_in_int = (ram_addr_out[ADDR_BUS_WIDTH - 1:13]) ? ram_in : data_in_int_tmp;
	case(ram_addr_out[1:0])
		6'b01: data_in_int = data_mem_in_int[15:8];
		6'b10: data_in_int = data_mem_in_int[31:16];
		6'b11: data_in_int = data_mem_in_int[31:24];
		default: data_in_int = data_mem_in_int;
	endcase
end

always @ (posedge clk)
begin
	if(rst)
	begin
		arith_inst_decode_fault_delayed_1c = 1'b0;
		core_inst_decode_fault_delayed_no_c_1c = 1'b0;
		core_stall_delayed_1c = 1'b0;
	end
	else
	begin
		arith_inst_decode_fault_delayed_1c = arith_inst_decode_fault;
		core_inst_decode_fault_delayed_no_c_1c = core_inst_decode_fault_no_c;
		core_stall_delayed_1c = core_stall_int;
	end
end

wire RS1_EQ_RS2;// = rs1 == rs2;
wire RS1_LT_RS2;// = $signed(rs1) < $signed(rs2);
wire RS1_LTU_RS2;// = rs1 < rs2;

/**************** Interrupt instance  ********************/
`define INT_BUS_SIZE (VECTOR_INT_TABLE_SIZE > 127) ? 8 : \
							(VECTOR_INT_TABLE_SIZE > 63) ? 7 : \
							(VECTOR_INT_TABLE_SIZE > 31) ? 6 : \
							(VECTOR_INT_TABLE_SIZE > 15) ? 5 : \
							(VECTOR_INT_TABLE_SIZE > 7) ? 4 : \
							(VECTOR_INT_TABLE_SIZE > 3) ? 3 : \
							(VECTOR_INT_TABLE_SIZE > 1) ? 2 : 1
 
wire [`INT_BUS_SIZE - 1 : 0]current_int_vect_request;
reg [`INT_BUS_SIZE - 1 : 0]current_int_vect_registered;
reg [`INT_BUS_SIZE - 1 : 0]IRN;

wire int_request;
reg int_registered;

int_encoder_dual_bus # (
	.VECTOR_INT_TABLE_SIZE(VECTOR_INT_TABLE_SIZE)
	)int_encoder_inst(
	.rst(rst),
	.int_sig_in(int_sig),
	.int_request(int_request),
	.int_vect(current_int_vect_request)
	);

always @ *
begin
	if(rst)
	begin
		instruction = 32'h00000000;
	end
	else 
	if(EXTENSION_C == "TRUE" && SYNCHRONOUS_ROM == "FALSE")
	begin/* C ISA decompressor. */
		if(~core_stall_int & ~STAGE_2)
		begin
			casex(pgm_inst_in[15:0])
			16'b00000000000xxx00: instruction = 32'h00000000; //																																										!C.ADDI4SPN. 
			16'b000xxxxxxxxxxx00: instruction = {{2{1'b0}}, {pgm_inst_in[10:7], pgm_inst_in[12:11], pgm_inst_in[5], pgm_inst_in[6], 2'h0}, 5'h2, 3'h0, {2'b01, pgm_inst_in[4:2]}, 7'b0010011}; //														C.ADDI4SPN. 
			16'b010xxxxxxxxxxx00: instruction = {6'h0, pgm_inst_in[5], pgm_inst_in[12:10], pgm_inst_in[6], 2'b0, {2'b01, pgm_inst_in[9:7]}, 3'b010, {2'b01, pgm_inst_in[4:2]}, 7'b0000011}; //															C.LW
			16'b110xxxxxxxxxxx00: instruction = {5'h0, pgm_inst_in[5], pgm_inst_in[12], {2'b01, pgm_inst_in[4:2]}, {2'b01, pgm_inst_in[9:7]}, 3'b010, {pgm_inst_in[11:10], pgm_inst_in[6], 2'b0}, 7'b0100011}; //											C.SW
			16'b000xxxxxxxxxxx01: instruction = {{7{pgm_inst_in[12]}}, pgm_inst_in[6:2], pgm_inst_in[11:7], 3'h0, pgm_inst_in[11:7], 7'b0010011}; //																								C.ADDI/C.ADDIW. 
			16'b011x00010xxxxx01: instruction = {{3{pgm_inst_in[12]}}, {pgm_inst_in[4:3], pgm_inst_in[5], pgm_inst_in[2], pgm_inst_in[6], 4'h0}, 5'h2, 3'h0, 5'h2, 7'b0010011}; //																		C.ADDI16SP. 
			16'b011xxxxxxxxxxx01: instruction = {{{12{pgm_inst_in[12]}}, pgm_inst_in[6:2]}, pgm_inst_in[11:7], 7'b0110111};// 																													C.LUI
			16'b010xxxxxxxxxxx01: instruction = {{{7{pgm_inst_in[12]}}, pgm_inst_in[6:2]}, 5'h0, 3'h0, pgm_inst_in[11:7], 7'b0010011};// 																										C.LI
			16'b100011xxx1xxxx01: instruction = {7'b0000000, {2'b01, pgm_inst_in[4:2]}, {2'b01, pgm_inst_in[9:7]}, {2'b11, pgm_inst_in[5]}, {2'b01, pgm_inst_in[9:7]}, 7'b0110011};//																OR/AND
			16'b100011xxx00xxx01: instruction = {7'b0100000, {2'b01, pgm_inst_in[4:2]}, {2'b01, pgm_inst_in[9:7]}, 3'b000, {2'b01, pgm_inst_in[9:7]}, 7'b0110011};//																				SUB
			16'b100x10xxxxxxxx01: instruction = {{6{pgm_inst_in[12]}}, {pgm_inst_in[12], pgm_inst_in[6:2]}, {2'b01, pgm_inst_in[9:7]}, 3'b111, {2'b01, pgm_inst_in[9:7]}, 7'b0010011};//																ANDI
			16'b10000xxxxxxxxx01: instruction = {{1'h0, pgm_inst_in[10], 5'h0}, pgm_inst_in[6:2], {2'b01, pgm_inst_in[9:7]}, 3'b101, {2'b01, pgm_inst_in[9:7]}, 7'b0010011}; //																		SRAI/SRLI. 
			16'b11xxxxxxxxxxxx01: instruction = {pgm_inst_in[12], {3{pgm_inst_in[12]}}, {pgm_inst_in[6:5], pgm_inst_in[2]}, 5'h0, {2'b01, pgm_inst_in[9:7]}, {2'b00, pgm_inst_in[13]}, {pgm_inst_in[11:10], pgm_inst_in[4:3]}, pgm_inst_in[12], 7'b1100011};// 	C.BEQZ/C.BNEZ
			16'bx01xxxxxxxxxxx01: instruction = {pgm_inst_in[12], {pgm_inst_in[8], pgm_inst_in[10:9], pgm_inst_in[6], pgm_inst_in[7], pgm_inst_in[2], pgm_inst_in[11], pgm_inst_in[5:3], pgm_inst_in[12], {8{pgm_inst_in[12]}}}, {4'h0, ~pgm_inst_in[15]}, 7'b1101111};//C.J/C.JAL
			16'b000xxxxxxxxxxx10: instruction = {7'h0, {pgm_inst_in[12], pgm_inst_in[6:2]}, pgm_inst_in[11:7], 3'b001, pgm_inst_in[11:7], 7'b0010011};
			16'b100xxxxxx0000010: instruction = {12'h0, pgm_inst_in[11:7], 3'h0, {4'h0, pgm_inst_in[12]}, 7'b1100111};// 																														C.JR/C.JALR : RET
			16'b1000xxxxxxxxxx10: instruction = {7'b0000000, pgm_inst_in[6:2], 5'h0, 3'b000, pgm_inst_in[11:7], 7'b0110011};//																												C.MV
			16'b110xxxxxxxxxxx10: instruction = {4'h0, {pgm_inst_in[8:7], pgm_inst_in[12]}, pgm_inst_in[6:2], 5'h2, 3'b010, {pgm_inst_in[11:9], 2'h0}, 7'b0100011}; // 																				C.SWSP
			16'b010xxxxxxxxxxx10: instruction = {6'h0, {pgm_inst_in[3:2], pgm_inst_in[12], pgm_inst_in[6:4], 2'h0}, 5'h2, 3'b010, pgm_inst_in[11:7], 7'b0000011}; // 																				C.LWSP
			16'b1001xxxxxxxxxx10: instruction = {{7'b0000000}, pgm_inst_in[6:2], pgm_inst_in[11:7], 3'b000, pgm_inst_in[11:7], 7'b0110011};//																									ADD
			default: instruction = pgm_inst_in;
			endcase
		end
		else
		begin
			instruction = pgm_inst_in;
		end
	end
	else
	begin
		if(~core_stall_int & ~STAGE_2)
		begin
			instruction = pgm_inst_in;
		end
	end
end

/**************** !Interrupt instance  ********************/

reg [1:0]load_from_interrupt_stage_cnt;

always @ *
begin
/* ram_out */
	if(VECTOR_INT_TABLE_SIZE != 0)
	begin
		case({rs2a, load_from_interrupt_stage_cnt})
		7'b0000010: ram_out = IRN;
		7'b0000001: ram_out = RIP;
		default : ram_out = rs2;
		endcase
	end
	else
	begin
		ram_out = rs2;
	end
/* ram_addr_out */
	casex({core_stall_int, SKIP_EXECUTION, int_registered, STAGE_2, instruction})
	{1'bx, `RISC_V_ALU_INST_EXT_I_LOAD}: ram_addr_out = rs1 + {{21{instruction[31]}}, instruction[30:20]};
	default: ram_addr_out = rs1 + {{21{instruction[31]}}, instruction[30:25], instruction[11:7]};
	endcase
	
	core_inst_decode_fault_no_c = 1'b1;
/* rd */
	rd = rd_alu;
	casex({core_stall_int, SKIP_EXECUTION, STAGE_2, instruction})
	{(SYNCHRONOUS_RAM == "TRUE" ? 1'b1 : 1'b0), `RISC_V_ALU_INST_EXT_I_LOAD}: 
	begin
		case(instruction[14:12])
		`RISC_V_ALU_INST_EXT_I_LB: rd = {{24{data_in_int[7]}}, data_in_int[7:0]};
		`RISC_V_ALU_INST_EXT_I_LBU: rd = {24'h000000, data_in_int[7:0]};
		`RISC_V_ALU_INST_EXT_I_LH: rd = {{16{data_in_int[15]}}, data_in_int[15:0]};
		`RISC_V_ALU_INST_EXT_I_LHU: rd = {16'h0000, data_in_int[15:0]};
		`RISC_V_ALU_INST_EXT_I_LW: rd = data_in_int;
		endcase
		core_inst_decode_fault_no_c = 1'b0;
	end
	{1'b0, `RISC_V_ALU_INST_LUI}: 
	begin
		rd = {instruction[31:12], 12'h000};
		core_inst_decode_fault_no_c = 1'b0;
	end
	{1'b0, `RISC_V_ALU_INST_AUIPC}: 
	begin
		rd = PC_FOR_AUIPC_AND_REL_JMP + {instruction[31:12], 12'h000};
		core_inst_decode_fault_no_c = 1'b0;
	end
	{1'b0, `RISC_V_ALU_INST_JAL},
	{1'b0, `RISC_V_ALU_INST_JALR}: 
	begin
		rd = PC_FOR_JAL_JALR;
		core_inst_decode_fault_no_c = 1'b0;
	end
	endcase
/* data_write */
	ram_wr_w = 1'b0;
	ram_wr_h = 1'b0;
	ram_wr_b = 1'b0;
	data_write_sysreg_w = 1'b0;
	data_write_sysreg_h = 1'b0;
	data_write_sysreg_b = 1'b0;
	casex({core_stall_int, SKIP_EXECUTION, STAGE_2, instruction})
	{1'b0, `RISC_V_ALU_INST_EXT_I_STORE}: 
	begin
		case(instruction[13:12])
		`RISC_V_ALU_INST_EXT_I_SB: 
		begin
		  if(ram_addr_out[ADDR_BUS_WIDTH - 1:13]) 
		      ram_wr_b = 1'b1;
		  else
		      data_write_sysreg_b = 1'b1;
		end
		`RISC_V_ALU_INST_EXT_I_SH:  
		begin
		  if(|ram_addr_out[ADDR_BUS_WIDTH - 1:13]) 
		      ram_wr_h = 1'b1;
		  else
		      data_write_sysreg_h = 1'b1;
		end
		`RISC_V_ALU_INST_EXT_I_SW:  
		begin
		  if(ram_addr_out[ADDR_BUS_WIDTH - 1:13]) 
		      ram_wr_w = 1'b1;
		  else
		      data_write_sysreg_w = 1'b1;
		end
		endcase
		core_inst_decode_fault_no_c = 1'b0;
	end
	endcase
/* data_read */
/*	data_read_w = 1'b0;
	data_read_h = 1'b0;
	data_read_b = 1'b0;
	data_read_sysreg_w = 1'b0;
	data_read_sysreg_h = 1'b0;
	data_read_sysreg_b = 1'b0;
	casex({core_stall_int, SKIP_EXECUTION, STAGE_2, instruction})
	{(SYNCHRONOUS_RAM == "TRUE" ? 1'b1 : 1'b0), `RISC_V_ALU_INST_EXT_I_LOAD}:
	begin
		case(instruction[14:12])
		`RISC_V_ALU_INST_EXT_I_LB, 
		`RISC_V_ALU_INST_EXT_I_LBU:  
		begin
		  if(|data_addr[ADDR_BUS_WIDTH - 1:13]) 
		      data_read_b = 1'b1;
		  else
		      data_read_sysreg_b = 1'b1;
		end
		`RISC_V_ALU_INST_EXT_I_LH, 
		`RISC_V_ALU_INST_EXT_I_LHU:  
		begin
		  if(|data_addr[ADDR_BUS_WIDTH - 1:13]) 
		      data_read_h = 1'b1;
		  else
		      data_read_sysreg_h = 1'b1;
		end
		`RISC_V_ALU_INST_EXT_I_LW:  
		begin
		  if(|data_addr[ADDR_BUS_WIDTH - 1:13]) 
		      data_read_w = 1'b1;
		  else
		      data_read_sysreg_w = 1'b1;
		end
		endcase
		core_inst_decode_fault_no_c = 1'b0;
	end
	endcase*/
/* rdw */
	rdw = 1'b0;
	casex({core_stall_int, SKIP_EXECUTION, STAGE_2, instruction})
		{1'b0, `RISC_V_ALU_INST_EXT_I_R},
		{1'b0, `RISC_V_ALU_INST_EXT_I_I},
		{1'b0, `RISC_V_ALU_INST_EXT_I_R_W},
		{1'b0, `RISC_V_ALU_INST_EXT_M_R},
		{(SYNCHRONOUS_RAM == "TRUE" ? 1'b1 : 1'b0), `RISC_V_ALU_INST_EXT_I_LOAD},
		{1'b0, `RISC_V_ALU_INST_LUI},
		{1'b0, `RISC_V_ALU_INST_JAL},
		{1'b0, `RISC_V_ALU_INST_JALR},
		{1'b0, `RISC_V_ALU_INST_AUIPC}: 
		begin
			rdw = 1'b1;
			core_inst_decode_fault_no_c = 1'b0;
		end
	endcase
end

always @ (posedge clk)
begin
	if(rst)
	begin
		PC <= RESET_VECTOR;
		load_from_interrupt_stage_cnt <= 2'b00;
		int_registered <= 1'b0;
		ENTER_INTERRUPT <= 1'b0;
		STAGE_2 <= 1'b0;
		SKIP_EXECUTION <= 1'b0;
		DELAY_INT_REGISTERING = 1'b0;
		core_inst_decode_fault <= 1'b1;
		PERFORMANCE_CNT <= 0;
		int_rst <= 'h00000000;
	end
	else
	begin
		if(~core_stall_int)
		begin
			core_inst_decode_fault <= 1'b1;
			SKIP_EXECUTION <= 1'b0;
			DELAY_INT_REGISTERING = 1'b0;
			STAGE_2 <= 1'b0;
			ENTER_INTERRUPT = 1'b0;
			if(VECTOR_INT_TABLE_SIZE != 0)
			begin
				int_rst <= 'h00000000;
			end
			PC <= PC + ((EXTENSION_C == "TRUE" && SYNCHRONOUS_ROM == "FALSE") ? (pgm_inst_in[1:0] == 2'b11 ? 4 : 2) : 4);
			PERFORMANCE_CNT <= PERFORMANCE_CNT + 1;
			if(rdw && rda == 0 && load_from_interrupt_stage_cnt != 2'b10 && VECTOR_INT_TABLE_SIZE != 0)
			begin
				RIP <= rd;
			end
/* STAGE_2 */
			casex({ENTER_INTERRUPT, SKIP_EXECUTION, STAGE_2, instruction})
			{1'b0, 1'b0, `RISC_V_ALU_INST_EXT_I_LOAD}: 
			begin
				core_inst_decode_fault <= 1'b0;
				if(SYNCHRONOUS_RAM == "TRUE")
					STAGE_2 <= 1'b1;
				DELAY_INT_REGISTERING = 1'b1;
			end
			endcase
/* PC */
			casex({ENTER_INTERRUPT, SKIP_EXECUTION, STAGE_2, instruction})
			{1'b0, 1'b0, `RISC_V_ALU_INST_EXT_I_LOAD}: 
			begin
				core_inst_decode_fault <= 1'b0;
				PC <= PC;
				PERFORMANCE_CNT <= PERFORMANCE_CNT;
			end
			{1'b0, 1'b0, `RISC_V_ALU_INST_JAL}: 
			begin
				core_inst_decode_fault <= 1'b0;
				PC <= (PC == RESET_VECTOR ? ((SYNCHRONOUS_ROM == "TRUE") ? PC_FOR_JAL_JALR : PC_FOR_AUIPC_AND_REL_JMP) : PC_FOR_AUIPC_AND_REL_JMP) + {{12{instruction[31]}}, instruction[19:12], instruction[20], instruction[30:21], 1'b0};
			end
			{1'b0, 1'b0, `RISC_V_ALU_INST_JALR}: 
			begin
				core_inst_decode_fault <= 1'b0;
				PC <= {{21{instruction[31]}}, instruction[30:20]} + {rs1[31:1], 1'b0};
			end
			{1'b0, 1'b0, `RISC_V_ALU_INST_MRET}: 
			begin
				core_inst_decode_fault <= 1'b0;
				if(VECTOR_INT_TABLE_SIZE != 0)
				begin
					PC <= RIP;
				end
			end
			{1'b0, 1'b0, `RISC_V_ALU_INST_BEQ_BNE}: 
			begin
				core_inst_decode_fault <= 1'b0;
				if(RS1_EQ_RS2 ^ instruction[12])
					PC <= PC_FOR_AUIPC_AND_REL_JMP + {{21{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8], 1'b0};
			end
			{1'b0, 1'b0, `RISC_V_ALU_INST_BLT_BGE}: 
			begin
				core_inst_decode_fault <= 1'b0;
				if(RS1_LT_RS2 ^ instruction[12])
					PC <= PC_FOR_AUIPC_AND_REL_JMP + {{21{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8], 1'b0};
			end
			{1'b0, 1'b0, `RISC_V_ALU_INST_BLTU_BGEU}: 
			begin
				core_inst_decode_fault <= 1'b0;
				if(RS1_LTU_RS2 ^ instruction[12])
					PC <= PC_FOR_AUIPC_AND_REL_JMP + {{21{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8], 1'b0};
			end
			endcase
/* SKIP_EXECUTION */
			casex({ENTER_INTERRUPT, SKIP_EXECUTION, STAGE_2, instruction})
			{1'b0, 1'b0, `RISC_V_ALU_INST_JAL},
			{1'b0, 1'b0, `RISC_V_ALU_INST_JALR}: 
			begin
				core_inst_decode_fault <= 1'b0;
				if(SYNCHRONOUS_ROM == "TRUE")
					SKIP_EXECUTION <= 1'b1;
				DELAY_INT_REGISTERING = 1'b1;
			end
			{1'b0, 1'b0, `RISC_V_ALU_INST_MRET}: 
			begin
				core_inst_decode_fault <= 1'b0;
				if(VECTOR_INT_TABLE_SIZE != 0)
				begin
					if(SYNCHRONOUS_ROM == "TRUE")
						SKIP_EXECUTION <= 1'b1;
					DELAY_INT_REGISTERING = 1'b1;
					//CSR_UIE <= 1'b1;
				end
			end
			{1'b0, 1'b0, `RISC_V_ALU_INST_BEQ_BNE}: 
			begin
				core_inst_decode_fault <= 1'b0;
				if(RS1_EQ_RS2 ^ instruction[12])
				begin
					if(SYNCHRONOUS_ROM == "TRUE")
						SKIP_EXECUTION <= 1'b1;
					DELAY_INT_REGISTERING = 1'b1;
				end
			end
			{1'b0, 1'b0, `RISC_V_ALU_INST_BLT_BGE}: 
			begin
				core_inst_decode_fault <= 1'b0;
				if(RS1_LT_RS2 ^ instruction[12])
				begin
					if(SYNCHRONOUS_ROM == "TRUE")
						SKIP_EXECUTION <= 1'b1;
					DELAY_INT_REGISTERING = 1'b1;
				end
			end
			{1'b0, 1'b0, `RISC_V_ALU_INST_BLTU_BGEU}: 
			begin
				core_inst_decode_fault <= 1'b0;
				if(RS1_LTU_RS2 ^ instruction[12])
				begin
					if(SYNCHRONOUS_ROM == "TRUE")
						SKIP_EXECUTION <= 1'b1;
					DELAY_INT_REGISTERING = 1'b1;
				end
			end
			endcase
/* load_from_interrupt */
			casex({ENTER_INTERRUPT, SKIP_EXECUTION, STAGE_2, instruction})
			{1'b0, 1'b0, `RISC_V_ALU_INST_EXT_I_STORE}: 
			begin
				core_inst_decode_fault <= 1'b0;
				case(instruction[13:12])
				`RISC_V_ALU_INST_EXT_I_SW:
				begin
					if(VECTOR_INT_TABLE_SIZE != 0)
					begin
						if(rs2a == 5'h00 && load_from_interrupt_stage_cnt) 
						begin
							load_from_interrupt_stage_cnt <= load_from_interrupt_stage_cnt - 1;
						end
					end
				end
				endcase
			end
			endcase
/* Set "Interrupt flag reset & unlock CALL address insert" */ /*************************************************************/
			if(VECTOR_INT_TABLE_SIZE != 0)
			begin
				if(int_request && ~int_registered/* && ~STATE_2*/ && ~DELAY_INT_REGISTERING && reg_mstatus[`REG_MSTATUS_MIE_bp]/* && ~&int_rst*/ && load_from_interrupt_stage_cnt == 0)
				begin
					int_registered = 1'b1;
					current_int_vect_registered <= current_int_vect_request;
					int_rst <= 1'b1 << (current_int_vect_request - 1);
					RIP <= PC_FOR_JAL_JALR;
					IRN <= current_int_vect_request - 1;
					if(SYNCHRONOUS_ROM == "TRUE")
						SKIP_EXECUTION <= 1'b1;
					ENTER_INTERRUPT = 1'b1;
					load_from_interrupt_stage_cnt <= 2'b10;
					PC <= reg_mscratch + {current_int_vect_request, 2'h0};
				end
				else
					int_registered = 1'b0;
			end
			if(VECTOR_INT_TABLE_SIZE != 0)
			begin
				if(load_from_interrupt_stage_cnt == 2'b01) 
				begin
					load_from_interrupt_stage_cnt <= load_from_interrupt_stage_cnt - 1;
				end
			end
		end
	end
end

risc_v_alu_lite #
	(
	.PLATFORM(PLATFORM),
	.EXTENSION_M(EXTENSION_M),
	.EXTENSION_MDIV(EXTENSION_MDIV)
	) risc_v_alu_lite_inst(
	.clk(clk),
	.instruction(instruction),
	.rs1(rs1),
	.rs2(rs2),
	.rd(rd_alu),
	.rs1_eq_rs2(RS1_EQ_RS2),
	.rs1_lt_rs2(RS1_LT_RS2),
	.rs1_ltu_rs2(RS1_LTU_RS2),
	.arith_inst_decode_fault(arith_inst_decode_fault)

    );

regs #
	(
	.PLATFORM(PLATFORM),
	.NEG_CLK_REG_OUTS(NEG_CLK_REG_OUTS)
	)regs_inst(
	.clk(clk),
	.rs1a(rs1a),
	.rs1(rs1),
	.rs1r(1'b1),
	.rs2a(rs2a),
	.rs2(rs2),
	.rs2r(1'b1),
	.rda(rda),
	.rd(rd),
	.rdw(rdw)
	);
endmodule
