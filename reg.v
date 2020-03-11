`timescale 1ns / 1ps

/*
 * This IP is the register implementation for the RISCv core.
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

module regs #
	(
	parameter PLATFORM = "XILINX",
	parameter NEG_CLK_REG_OUTS = "FALSE"
	)(
	input clk,
	input [4:0]rs1a,
	output [31:0]rs1,
	input rs1r,
	input [4:0]rs2a,
	output [31:0]rs2,
	input rs2r,
	input [4:0]rda,
	input [31:0]rd,
	input rdw
	);

(* ram_style="block" *)
reg [31:0]REG[0:31];

integer clear_cnt;
initial 
begin
	for(clear_cnt = 0; clear_cnt < 32; clear_cnt = clear_cnt + 1)
	begin:CLEAR
		REG[clear_cnt] <= 32'h00000000;
	end
end

always @ (posedge clk)
begin
	if(rdw)
		REG[rda] <= rd;
end

reg [31:0]rs1_int;
reg [31:0]rs2_int;

generate
if(NEG_CLK_REG_OUTS == "TRUE")
begin
	always @ (negedge clk)
	begin
		rs1_int <= REG[rs1a];
		rs2_int <= REG[rs2a];
	end
end
else
begin
	always @ *
	begin
		rs1_int = REG[rs1a];
		rs2_int = REG[rs2a];
	end
end
endgenerate
assign rs1 = (rs1r && rs1a != 0) ? rs1_int : 32'h00000000;
assign rs2 = (rs2r && rs2a != 0) ? rs2_int : 32'h00000000;


endmodule
