/*
 * This IP is the Lite RISC-V RV32I single bus simulation file.
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


module riscvLiteSim();

reg clk = 1;
reg rst = 0;
always	#(1)	clk	<=	~clk;
wire [31:0]led;
reg [31:0]sw = 0;

initial begin
	wait(clk);
	wait(~clk);
	rst = 0;
	wait(~clk);
	wait(clk);
	wait(~clk);
	rst = 1;
	#400000;
	$finish;
end

riscv_single_bus_top riscv_single_bus_top_inst(
	.rst(rst),
	.clk(clk),
	.LED(led),
	.SW(sw)
);

endmodule