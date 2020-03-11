/*
 * This IP is the Lite RISC-V RV32I ALU implementation.
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

`include "def-h.v"

module risc_v_alu_light #
	(
	parameter PLATFORM = "XILINX",
	parameter EXTENSION_M = "TRUE",
	parameter EXTENSION_MDIV = "FALSE"
	)(
	input clk,
	input [31:0]instruction,
	input [31:0]rs1,
	input [31:0]rs2,
	output reg[31:0]rd,
	output rs1_eq_rs2,
	output rs1_lt_rs2,
	output rs1_ltu_rs2,
	output reg arith_inst_decode_fault 
    );

reg [31:0]TMP;
reg [63:0]TMP1;
reg [63:0]TMP2;

function [31:0]SLL;
	input [31:0]val_in;
	input [4:0]shift;
	begin
		case(shift)
		5'h00: SLL = val_in;
		5'h01: SLL = {val_in[30:0], 1'h0};
		5'h02: SLL = {val_in[29:0], 2'h0};
		5'h03: SLL = {val_in[28:0], 3'h0};
		5'h04: SLL = {val_in[27:0], 4'h0};
		5'h05: SLL = {val_in[26:0], 5'h0};
		5'h06: SLL = {val_in[25:0], 6'h0};
		5'h07: SLL = {val_in[24:0], 7'h0};
		5'h08: SLL = {val_in[23:0], 8'h0};
		5'h09: SLL = {val_in[22:0], 9'h0};
		5'h0A: SLL = {val_in[21:0], 10'h0};
		5'h0B: SLL = {val_in[20:0], 11'h0};
		5'h0C: SLL = {val_in[19:0], 12'h0};
		5'h0D: SLL = {val_in[18:0], 13'h0};
		5'h0E: SLL = {val_in[17:0], 14'h0};
		5'h0F: SLL = {val_in[16:0], 15'h0};
		5'h10: SLL = {val_in[15:0], 16'h0};
		5'h11: SLL = {val_in[14:0], 17'h0};
		5'h12: SLL = {val_in[13:0], 18'h0};
		5'h13: SLL = {val_in[12:0], 19'h0};
		5'h14: SLL = {val_in[11:0], 20'h0};
		5'h15: SLL = {val_in[10:0], 21'h0};
		5'h16: SLL = {val_in[9:0], 22'h0};
		5'h17: SLL = {val_in[8:0], 23'h0};
		5'h18: SLL = {val_in[7:0], 24'h0};
		5'h19: SLL = {val_in[6:0], 25'h0};
		5'h1A: SLL = {val_in[5:0], 26'h0};
		5'h1B: SLL = {val_in[4:0], 27'h0};
		5'h1C: SLL = {val_in[3:0], 28'h0};
		5'h1D: SLL = {val_in[2:0], 29'h0};
		5'h1E: SLL = {val_in[1:0], 30'h0};
		5'h1F: SLL = {val_in[0], 31'h0};
		endcase
	end
endfunction

function [31:0]SRAL;
	input [31:0]val_in;
	input [4:0]shift;
	input arithmetic;
	begin
		case(shift)
		5'h00: SRAL = val_in;
		5'h01: SRAL = {{1{arithmetic}}, val_in[31:1]};
		5'h02: SRAL = {{2{arithmetic}}, val_in[31:2]};
		5'h03: SRAL = {{3{arithmetic}}, val_in[31:3]};
		5'h04: SRAL = {{4{arithmetic}}, val_in[31:4]};
		5'h05: SRAL = {{5{arithmetic}}, val_in[31:5]};
		5'h06: SRAL = {{6{arithmetic}}, val_in[31:6]};
		5'h07: SRAL = {{7{arithmetic}}, val_in[31:7]};
		5'h08: SRAL = {{8{arithmetic}}, val_in[31:8]};
		5'h09: SRAL = {{9{arithmetic}}, val_in[31:9]};
		5'h0A: SRAL = {{10{arithmetic}}, val_in[31:10]};
		5'h0B: SRAL = {{11{arithmetic}}, val_in[31:11]};
		5'h0C: SRAL = {{12{arithmetic}}, val_in[31:12]};
		5'h0D: SRAL = {{13{arithmetic}}, val_in[31:13]};
		5'h0E: SRAL = {{14{arithmetic}}, val_in[31:14]};
		5'h0F: SRAL = {{15{arithmetic}}, val_in[31:15]};
		5'h10: SRAL = {{16{arithmetic}}, val_in[31:16]};
		5'h11: SRAL = {{17{arithmetic}}, val_in[31:17]};
		5'h12: SRAL = {{18{arithmetic}}, val_in[31:18]};
		5'h13: SRAL = {{19{arithmetic}}, val_in[31:19]};
		5'h14: SRAL = {{20{arithmetic}}, val_in[31:20]};
		5'h15: SRAL = {{21{arithmetic}}, val_in[31:21]};
		5'h16: SRAL = {{22{arithmetic}}, val_in[31:22]};
		5'h17: SRAL = {{23{arithmetic}}, val_in[31:23]};
		5'h18: SRAL = {{24{arithmetic}}, val_in[31:24]};
		5'h19: SRAL = {{25{arithmetic}}, val_in[31:25]};
		5'h1A: SRAL = {{26{arithmetic}}, val_in[31:26]};
		5'h1B: SRAL = {{27{arithmetic}}, val_in[31:27]};
		5'h1C: SRAL = {{28{arithmetic}}, val_in[31:28]};
		5'h1D: SRAL = {{29{arithmetic}}, val_in[31:29]};
		5'h1E: SRAL = {{30{arithmetic}}, val_in[31:30]};
		5'h1F: SRAL = {{31{arithmetic}}, val_in[31]};
		endcase
	end
endfunction

reg SRAL_INPUT_BIT;
wire [31:0]SRAL_OUTPUT = SRAL(rs1, TMP[4:0], SRAL_INPUT_BIT);

reg [63:0]mul_result_int;
reg [63:0]div_result_int;

generate
always @ *
begin
	if(PLATFORM == "XILINX")
	begin
		//(* use_dsp48 = "yes" *)
		//mul_result_int = rs1 * rs2;
		(* use_dsp48 = "yes" *)
		div_result_int = TMP1 / TMP2;
	end
	else
	begin
		//(* use_dsp48 = "yes" *)
		//mul_result_int = rs1 * rs2;
		(* use_dsp48 = "yes" *)
		div_result_int = TMP1 / TMP2;
	end
end
endgenerate

assign rs1_eq_rs2 = rs1 == rs2;
assign rs1_lt_rs2 = $signed(rs1) < $signed(TMP);
assign rs1_ltu_rs2 = rs1 < TMP;

/*wire [63:0]mul_result_u_int = rs1 * TMP;
wire signed [31:0]mul_s_a = rs1;
wire signed [31:0]mul_s_b = TMP;
wire signed [32:0]mul_su_b = {1'b0, TMP};
wire signed [63:0]mul_result_s_int = mul_s_a * mul_s_b;
wire signed [63:0]mul_result_s_u_int = mul_s_a * mul_su_b;*/

reg add_sub_int;
wire [31:0]RD_ADDI_ADDIW_ADD_ADDW_SUB_SUBW;
wire signed [65:0]mul_result_s_int;
wire signed [32:0]mul_s_a;
wire signed [32:0]mul_s_b;
generate
if(PLATFORM == "iCE40UP")
begin
pmi_addsub
#(
  .pmi_data_width   (32), // integer
  .pmi_sign         ("off"), // "on"|"off"
  .pmi_family       ("common"), // "common"
  .module_type      ("pmi_add")  // "pmi_add"
) add_inst (
  .DataA    (rs1),  // I:
  .DataB    (TMP),  // I:
  .Cin      (1'b0),  // I:
  .Add_Sub  (add_sub_int),  // I:
  .Result   (RD_ADDI_ADDIW_ADD_ADDW_SUB_SUBW),  // O:
  .Cout     ( ),  // O:
  .Overflow ( )   // O:
);
if(EXTENSION_M == "TRUE")
begin
MUL_DEV MUL_DEV_inst(
		.clk_i(clk), 
        .clk_en_i(1'b1),
		.data_a_i(TMP1[32:0]), 
        .data_b_i(TMP2[32:0]), 
        .result_o(mul_result_s_int)
		) ;
end
end
else
begin
assign RD_ADDI_ADDIW_ADD_ADDW_SUB_SUBW = rs1 + TMP;
assign mul_s_a = TMP1;
assign mul_s_b = TMP2;
assign mul_result_s_int = mul_s_a * mul_s_b;
end
endgenerate

wire [31:0]RD_SLL_SLLW_SLLI_SLLIW = SLL(rs1, TMP[4:0]);
wire [31:0]RD_SLTI_SLT = {31'h0, rs1_lt_rs2};
wire [31:0]RD_SLTIU_SLTU = {31'h0, rs1_ltu_rs2};
wire [31:0]RD_SRL_SRLW_SRLI_SRLIW_SRA_SRAW_SRAI_SRAIW = SRAL_OUTPUT;
wire [31:0]RD_XORI_XOR = rs1 ^ TMP;
wire [31:0]RD_ORI_OR = rs1 | TMP;
wire [31:0]RD_ANDI_AND = rs1 & TMP;

always @ *
begin
	arith_inst_decode_fault = 1'b1;
/* Set "TMP" */ /*************************************************************/
	casex(instruction)
	`RISC_V_ALU_INST_EXT_I_I: TMP = {{21{instruction[31]}}, instruction[30:20]};
	default: TMP = rs2;
	endcase
/* Negate rs2 for substraction */ /*************************************************************/
	if(PLATFORM == "iCE40UP")
	begin
		add_sub_int = 1'b0;
		casex({instruction[30], instruction[25], instruction[14:12], instruction[6:0]})
		`RISC_V_ALU_INST_EXT_I_SUB_SUBW: add_sub_int = 1'b1;
		endcase
	end
	else
	begin
		casex({instruction[30], instruction[25], instruction[14:12], instruction[6:0]})
		`RISC_V_ALU_INST_EXT_I_SUB_SUBW: TMP = 0 - TMP;
		endcase
	end
/* Set "SRAL_INPUT_BIT" */ /*************************************************************/
	casex({instruction[30], instruction[25], instruction[14:12], instruction[6:0]})
	`RISC_V_ALU_INST_EXT_I_SRA_SRAW_SRAI_SRAIW: SRAL_INPUT_BIT <= rs1[31];
	/*`RISC_V_ALU_INST_EXT_I_SRL_SRLW_SRLI_SRLIW*/default: SRAL_INPUT_BIT <= 1'b0;
	endcase
/* Set "TMP1 & TMP2" */ /*************************************************************/
	casex({instruction[30], instruction[25], instruction[14:12], instruction[6:0]})
	`RISC_V_ALU_INST_EXT_M_MUL,
	`RISC_V_ALU_INST_EXT_M_MULH: 
	begin
		if(EXTENSION_M == "TRUE")
		begin
			arith_inst_decode_fault = 1'b0;
			TMP1 = {rs1[31:0], 1'h0};
			TMP2 = {rs2[31:0], 1'h0};
		end
	end
	`RISC_V_ALU_INST_EXT_M_MULHSU: 
	begin
		if(EXTENSION_M == "TRUE")
		begin
			arith_inst_decode_fault = 1'b0;
			TMP1 = {rs1[31:0], 1'h0};
			TMP2 = {1'h0, rs2[31:0]};
		end
	end
	`RISC_V_ALU_INST_EXT_M_MULHU: 
	begin
		if(EXTENSION_M == "TRUE")
		begin
			arith_inst_decode_fault = 1'b0;
			TMP1 = {1'h0, rs1[31:0]};
			TMP2 = {1'h0, rs2[31:0]};
		end
	end
	`RISC_V_ALU_INST_EXT_M_DIV: 
	begin
		if(EXTENSION_MDIV == "TRUE")
		begin
			arith_inst_decode_fault = 1'b0;
			TMP1 = {1'b0, rs1[30:0], 32'h00000000};
			TMP2 = {1'b0, rs2[30:0], 32'h00000000};
		end
	end
	`RISC_V_ALU_INST_EXT_M_DIVU: 
	begin
		if(EXTENSION_MDIV == "TRUE")
		begin
			arith_inst_decode_fault = 1'b0;
			TMP1 = {rs1, 32'h00000000};
			TMP2 = {rs2, 32'h00000000};
		end
	end
	`RISC_V_ALU_INST_EXT_M_REM: 
	begin
		if(EXTENSION_MDIV == "TRUE")
		begin
			arith_inst_decode_fault = 1'b0;
			TMP1 = {1'b0, rs1[30:0], 32'h00000000};
			TMP2 = {1'b0, rs2[30:0], 32'h00000000};
		end
	end
	`RISC_V_ALU_INST_EXT_M_REMU: 
	begin
		if(EXTENSION_MDIV == "TRUE")
		begin
			arith_inst_decode_fault = 1'b0;
			TMP1 = {rs1, 32'h00000000};
			TMP2 = {rs2, 32'h00000000};
		end
	end
	endcase
/* Set "rd" */ /*************************************************************/
	casex({instruction[30], instruction[25], instruction[14:12], instruction[6:0]})
	`RISC_V_ALU_INST_EXT_I_ADDI_ADDIW,
	`RISC_V_ALU_INST_EXT_I_ADD_ADDW,
	`RISC_V_ALU_INST_EXT_I_SUB_SUBW: 
	begin
		arith_inst_decode_fault = 1'b0;
		rd = RD_ADDI_ADDIW_ADD_ADDW_SUB_SUBW;//rs1 + TMP;
	end
	`RISC_V_ALU_INST_EXT_I_SLL_SLLW_SLLI_SLLIW: 
	begin
		arith_inst_decode_fault = 1'b0;
		rd = RD_SLL_SLLW_SLLI_SLLIW;//SLL(rs1, TMP[4:0]); 
	end
	`RISC_V_ALU_INST_EXT_I_SLTI,
	`RISC_V_ALU_INST_EXT_I_SLT: 
	begin
		arith_inst_decode_fault = 1'b0;
		rd = RD_SLTI_SLT;//{31'h0, rs1_lt_rs2};
	end
	`RISC_V_ALU_INST_EXT_I_SLTIU,
	`RISC_V_ALU_INST_EXT_I_SLTU: 
	begin
		arith_inst_decode_fault = 1'b0;
		rd = RD_SLTIU_SLTU;//{31'h0, rs1_ltu_rs2};
	end
	`RISC_V_ALU_INST_EXT_I_SRL_SRLW_SRLI_SRLIW,
	`RISC_V_ALU_INST_EXT_I_SRA_SRAW_SRAI_SRAIW: 
	begin
		arith_inst_decode_fault = 1'b0;
		rd = RD_SRL_SRLW_SRLI_SRLIW_SRA_SRAW_SRAI_SRAIW;//SRAL_OUTPUT;
	end
	`RISC_V_ALU_INST_EXT_I_XORI_XOR: 
	begin
		arith_inst_decode_fault = 1'b0;
		rd = RD_XORI_XOR;//rs1 ^ TMP;
	end
	`RISC_V_ALU_INST_EXT_I_ORI_OR: 
	begin
		arith_inst_decode_fault = 1'b0;
		rd = RD_ORI_OR;//rs1 | TMP;
	end
	`RISC_V_ALU_INST_EXT_I_ANDI_AND: 
	begin
		arith_inst_decode_fault = 1'b0;
		rd = RD_ANDI_AND;//rs1 & TMP;
	end
/*
 * M Extension
 */
	`RISC_V_ALU_INST_EXT_M_MUL: 
	begin
		if(EXTENSION_M == "TRUE")
		begin
			arith_inst_decode_fault = 1'b0;
			rd = mul_result_s_int[33:2];
		end
	end
	`RISC_V_ALU_INST_EXT_M_MULH: 
	begin
		if(EXTENSION_M == "TRUE")
		begin
			arith_inst_decode_fault = 1'b0;
			rd = mul_result_s_int[65:34];
		end
	end
	`RISC_V_ALU_INST_EXT_M_MULHSU: 
	begin
		if(EXTENSION_M == "TRUE")
		begin
			arith_inst_decode_fault = 1'b0;
			//rd = (rs1[31] ? ~mul_result_int[63:32] : mul_result_int[63:32]);
			rd = mul_result_s_int[64:33];
		end
	end
	`RISC_V_ALU_INST_EXT_M_MULHU: 
	begin
		if(EXTENSION_M == "TRUE")
		begin
			arith_inst_decode_fault = 1'b0;
			rd = mul_result_s_int[63:32];
		end
	end
	`RISC_V_ALU_INST_EXT_M_DIV: 
	begin
		if(EXTENSION_MDIV == "TRUE")
		begin
			arith_inst_decode_fault = 1'b0;
			rd = {TMP1[63] ^ TMP2[63], div_result_int[62:32]};
		end
	end
	`RISC_V_ALU_INST_EXT_M_DIVU: 
	begin
		if(EXTENSION_MDIV == "TRUE")
		begin
			arith_inst_decode_fault = 1'b0;
			rd = div_result_int[63:32];
		end
	end
	`RISC_V_ALU_INST_EXT_M_REM: 
	begin
		if(EXTENSION_MDIV == "TRUE")
		begin
			arith_inst_decode_fault = 1'b0;
			rd = {TMP1[63] ^ TMP2[63], div_result_int[31:1]};
		end
	end
	`RISC_V_ALU_INST_EXT_M_REMU: 
	begin
		if(EXTENSION_MDIV == "TRUE")
		begin
			arith_inst_decode_fault = 1'b0;
			rd = div_result_int[31:0];
		end
	end
	//default: rd = 32'h00000000;
	endcase
end


endmodule
