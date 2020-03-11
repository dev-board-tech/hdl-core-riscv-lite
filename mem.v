/*
 * This IP is the ROM and RAM memory implementationfor the RISCv core.
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

module ram #(
	parameter PLATFORM = "XILINX",
	parameter ADDR_BUS_LEN = 10,
	parameter RAM_PATH = "",
	parameter SYNCHRONOUS_OUTPUT = "TRUE",
	parameter RAM_MODE = "SRAM"// SRAM, BLOCK
	)(
	input clk,
	input [ADDR_BUS_LEN - 1:0]addr,
	input cs,
	output reg [31:0]out,
	input [31:0]in,
	input write_w,
	input write_h,
	input write_b
);
generate
if(PLATFORM == "XILINX")
begin
reg [31:0]TMP;
(* ram_style="block" *)
reg [7:0]mem_b0[((2**ADDR_BUS_LEN) / 4)-1:0];
(* ram_style="block" *)
reg [7:0]mem_b1[((2**ADDR_BUS_LEN) / 4)-1:0];
(* ram_style="block" *)
reg [7:0]mem_b2[((2**ADDR_BUS_LEN) / 4)-1:0];
(* ram_style="block" *)
reg [7:0]mem_b3[((2**ADDR_BUS_LEN) / 4)-1:0];


integer clear_cnt;
initial begin
if (RAM_PATH != "")
begin
	$readmemh({RAM_PATH, "_0.mem"}, mem_b0);
	$readmemh({RAM_PATH, "_1.mem"}, mem_b1);
	$readmemh({RAM_PATH, "_2.mem"}, mem_b2);
	$readmemh({RAM_PATH, "_3.mem"}, mem_b3);
end
else
begin
	for(clear_cnt = 0; clear_cnt < ((2**ADDR_BUS_LEN) / 4)-1; clear_cnt = clear_cnt + 1)
	begin:CLEAR_RAM
		mem_b0[clear_cnt] = 8'h00;
		mem_b1[clear_cnt] = 8'h00;
		mem_b2[clear_cnt] = 8'h00;
		mem_b3[clear_cnt] = 8'h00;
	end
end
end
reg cs_del;
reg [1:0]addr_del;
always @ (posedge clk)
begin
	cs_del <= cs;
	addr_del <= addr[1:0];
	if(write_w & cs)
	begin
		mem_b0[addr[ADDR_BUS_LEN-1:2]] <= in[7:0];
		mem_b1[addr[ADDR_BUS_LEN-1:2]] <= in[15:8];
		mem_b2[addr[ADDR_BUS_LEN-1:2]] <= in[23:16];
		mem_b3[addr[ADDR_BUS_LEN-1:2]] <= in[31:24];
	end
	else if(write_h & cs)
	begin
		case(addr[1:0])
		2'b00: 
		begin
			mem_b0[addr[ADDR_BUS_LEN-1:2]] <= in[7:0];
			mem_b1[addr[ADDR_BUS_LEN-1:2]] <= in[15:8];
		end
		2'b10: 
		begin
			mem_b2[addr[ADDR_BUS_LEN-1:2]] <= in[7:0];
			mem_b3[addr[ADDR_BUS_LEN-1:2]] <= in[15:8];
		end
		endcase
	end
	else if(write_b & cs)
	begin
		case(addr[1:0])
		2'h0: mem_b0[addr[ADDR_BUS_LEN-1:2]] <= in[7:0];
		2'h1: mem_b1[addr[ADDR_BUS_LEN-1:2]] <= in[7:0];
		2'h2: mem_b2[addr[ADDR_BUS_LEN-1:2]] <= in[7:0];
		2'h3: mem_b3[addr[ADDR_BUS_LEN-1:2]] <= in[7:0];
		endcase
	end
end

if(SYNCHRONOUS_OUTPUT == "TRUE")
begin
	always @ (posedge clk)
	begin
		TMP <= {mem_b3[addr[ADDR_BUS_LEN-1:2]], mem_b2[addr[ADDR_BUS_LEN-1:2]], mem_b1[addr[ADDR_BUS_LEN-1:2]], mem_b0[addr[ADDR_BUS_LEN-1:2]]};
	end
end
else
begin
	always @ *
	begin
		TMP = {mem_b3[addr[ADDR_BUS_LEN-1:2]], mem_b2[addr[ADDR_BUS_LEN-1:2]], mem_b1[addr[ADDR_BUS_LEN-1:2]], mem_b0[addr[ADDR_BUS_LEN-1:2]]};
	end
end

always @ *
begin
	out = 32'h00000000;
	if(cs)
		out = TMP;
end


end /* PLATFORM != "XILINX" */
else
if(PLATFORM == "LATTICE_MARCHXO2" || PLATFORM == "LATTICE_MARCHXO3L")
begin

wire [7:0]TMPO1_GEN[0 : (ADDR_BUS_LEN > 12 ? ADDR_BUS_LEN - 12 : 0)];
wire [7:0]TMPO2_GEN[0 : (ADDR_BUS_LEN > 12 ? ADDR_BUS_LEN - 12 : 0)];
wire [7:0]TMPO3_GEN[0 : (ADDR_BUS_LEN > 12 ? ADDR_BUS_LEN - 12 : 0)];
wire [7:0]TMPO4_GEN[0 : (ADDR_BUS_LEN > 12 ? ADDR_BUS_LEN - 12 : 0)];

reg [7:0]TMPI1;
reg [7:0]TMPI2;
reg [7:0]TMPI3;
reg [7:0]TMPI4;

wire [7:0]TMPO1;
wire [7:0]TMPO2;
wire [7:0]TMPO3;
wire [7:0]TMPO4;

reg WRITE1;
reg WRITE2;
reg WRITE3;
reg WRITE4;

always @ *
begin
	TMPI1 <= 8'h00;
	TMPI2 <= 8'h00;
	TMPI3 <= 8'h00;
	TMPI4 <= 8'h00;
	WRITE1 <= 1'b0;
	WRITE2 <= 1'b0;
	WRITE3 <= 1'b0;
	WRITE4 <= 1'b0;
	if(write_w & cs)
	begin
		TMPI1 <= in[7:0];
		TMPI2 <= in[15:8];
		TMPI3 <= in[23:16];
		TMPI4 <= in[31:24];
		WRITE1 <= 1'b1;
		WRITE2 <= 1'b1;
		WRITE3 <= 1'b1;
		WRITE4 <= 1'b1;
	end
	else if(write_h & cs)
	begin
		case(addr[1:0])
		2'b00: 
		begin
			TMPI1 <= in[7:0];
			TMPI2 <= in[15:8];
			WRITE1 <= 1'b1;
			WRITE2 <= 1'b1;
		end
		2'b10: 
		begin
			TMPI3 <= in[7:0];
			TMPI4 <= in[15:8];
			WRITE3 <= 1'b1;
			WRITE4 <= 1'b1;
		end
		endcase
	end
	else if(write_b & cs)
	begin
		case(addr[1:0])
		2'h0: 
		begin
			TMPI1 <= in[7:0];
			WRITE1 <= 1'b1;
		end
		2'h1: 
		begin
			TMPI2 <= in[7:0];
			WRITE2 <= 1'b1;
		end
		2'h2: 
		begin
			TMPI3 <= in[7:0];
			WRITE3 <= 1'b1;
		end
		2'h3: 
		begin
			TMPI4 <= in[7:0];
			WRITE4 <= 1'b1;
		end
		endcase
	end
end

sram sram1_int (
	.Clock(clk),
	.ClockEn(1'b1),
	.Reset(1'b0),
	.WE(WRITE1),
	.Address(addr[ADDR_BUS_LEN-1:2]),
	.Data(TMPI1),
    .Q(TMPO1)
);

sram sram2_int (
	.Clock(clk),
	.ClockEn(1'b1),
	.Reset(1'b0),
	.WE(WRITE2),
	.Address(addr[ADDR_BUS_LEN-1:2]),
	.Data(TMPI2),
    .Q(TMPO2)
);

sram sram3_int (
	.Clock(clk),
	.ClockEn(1'b1),
	.Reset(1'b0),
	.WE(WRITE3),
	.Address(addr[ADDR_BUS_LEN-1:2]),
	.Data(TMPI3),
    .Q(TMPO3)
);

sram sram4_int (
	.Clock(clk),
	.ClockEn(1'b1),
	.Reset(1'b0),
	.WE(WRITE4),
	.Address(addr[ADDR_BUS_LEN-1:2]),
	.Data(TMPI4),
    .Q(TMPO4)
);


always @ *
begin
	out <= 32'h00000000;
	if(cs)
		out <= {TMPO4, TMPO3, TMPO2, TMPO1};
end

end/* PLATFORM != "LATTICE_MARCHXO2" || PLATFORM != "LATTICE_MARCHXO3L" */
else
if(PLATFORM == "iCE40UP")
begin
wire [31:0]TMP;

if(RAM_MODE == "SRAM")
begin
reg [31:0]TMP_WR;
reg [7:0]WR_MASK;

always @ *
begin
	WR_MASK = 8'b00000000;
	TMP_WR = 32'h00000000;
	if(write_w)
	begin
		TMP_WR = in;
		WR_MASK = 8'b11111111;
	end
	else if(write_h)
	begin
		case(addr[1])
		1'b0:
		begin
			TMP_WR = in[15:0];
			WR_MASK = 8'b00001111;
		end
		1'b0:
		begin
			TMP_WR[31:16] = in[15:0];
			WR_MASK = 8'b11110000;
		end
		endcase
	end
	else if(write_b)
	begin
		case(addr[1:0])
		2'b00:
		begin
			TMP_WR = in[7:0];
			WR_MASK = 8'b00000011;
		end
		2'b01:
		begin
			TMP_WR[15:8] = in[7:0];
			WR_MASK = 8'b00001100;
		end
		2'b10:
		begin
			TMP_WR[23:16] = in[7:0];
			WR_MASK = 8'b00001100;
		end
		2'b11:
		begin
			TMP_WR[31:24] = in[7:0];
			WR_MASK = 8'b11000000;
		end
		endcase
	end
end

wire we = |{write_w, write_h, write_b};

SP256K  ramfn_inst1(
	.DI(TMP_WR[15:0]),
	.AD(addr[ADDR_BUS_LEN - 1: 2]),
	.MASKWE({WR_MASK[3:2], WR_MASK[1:0]}),
	.WE(we),
	.CS(cs),
	.CK(clk),
	.STDBY(1'b0),
	.SLEEP(1'b0),
	.PWROFF_N(1'b1),
	.DO(TMP[15:0])
);

SP256K  ramfn_inst2(
	.DI(TMP_WR[31:16]),
	.AD(addr[ADDR_BUS_LEN - 1: 2]),
	.MASKWE({WR_MASK[7:6], WR_MASK[5:4]}),
	.WE(we),
	.CS(cs),
	.CK(clk),
	.STDBY(1'b0),
	.SLEEP(1'b0),
	.PWROFF_N(1'b1),
	.DO(TMP[31:16])
);

end //RAM_MODE != "SRAM"
else
begin

wire [7:0]TMPO1_GEN[0 : (ADDR_BUS_LEN > 12 ? ADDR_BUS_LEN - 12 : 0)];
wire [7:0]TMPO2_GEN[0 : (ADDR_BUS_LEN > 12 ? ADDR_BUS_LEN - 12 : 0)];
wire [7:0]TMPO3_GEN[0 : (ADDR_BUS_LEN > 12 ? ADDR_BUS_LEN - 12 : 0)];
wire [7:0]TMPO4_GEN[0 : (ADDR_BUS_LEN > 12 ? ADDR_BUS_LEN - 12 : 0)];

reg [7:0]TMPI1;
reg [7:0]TMPI2;
reg [7:0]TMPI3;
reg [7:0]TMPI4;

wire [7:0]TMPO1;
wire [7:0]TMPO2;
wire [7:0]TMPO3;
wire [7:0]TMPO4;

reg WRITE1;
reg WRITE2;
reg WRITE3;
reg WRITE4;

always @ *
begin
	TMPI1 <= 8'h00;
	TMPI2 <= 8'h00;
	TMPI3 <= 8'h00;
	TMPI4 <= 8'h00;
	WRITE1 <= 1'b0;
	WRITE2 <= 1'b0;
	WRITE3 <= 1'b0;
	WRITE4 <= 1'b0;
	if(write_w & cs)
	begin
		TMPI1 <= in[7:0];
		TMPI2 <= in[15:8];
		TMPI3 <= in[23:16];
		TMPI4 <= in[31:24];
		WRITE1 <= 1'b1;
		WRITE2 <= 1'b1;
		WRITE3 <= 1'b1;
		WRITE4 <= 1'b1;
	end
	else if(write_h & cs)
	begin
		case(addr[1:0])
		2'b00: 
		begin
			TMPI1 <= in[7:0];
			TMPI2 <= in[15:8];
			WRITE1 <= 1'b1;
			WRITE2 <= 1'b1;
		end
		2'b10: 
		begin
			TMPI3 <= in[7:0];
			TMPI4 <= in[15:8];
			WRITE3 <= 1'b1;
			WRITE4 <= 1'b1;
		end
		endcase
	end
	else if(write_b & cs)
	begin
		case(addr[1:0])
		2'h0: 
		begin
			TMPI1 <= in[7:0];
			WRITE1 <= 1'b1;
		end
		2'h1: 
		begin
			TMPI2 <= in[7:0];
			WRITE2 <= 1'b1;
		end
		2'h2: 
		begin
			TMPI3 <= in[7:0];
			WRITE3 <= 1'b1;
		end
		2'h3: 
		begin
			TMPI4 <= in[7:0];
			WRITE4 <= 1'b1;
		end
		endcase
	end
end

pmi_ram_dq
#(
	.pmi_addr_depth       (2 << ADDR_BUS_LEN), // integer       
	.pmi_addr_width       (ADDR_BUS_LEN), // integer       
	.pmi_data_width       (8), // integer
	.pmi_regmode          ("noreg"), // "reg"|"noreg"
	.pmi_gsr              ("enable"), // "enable"|"disable"
	.pmi_resetmode        ("sync"), // "async"|"sync"
	.pmi_optimization     ("speed"), // "area"|"speed"
	.pmi_init_file        (""), // string
	.pmi_init_file_format ("hex"), // "binary"|"hex"
	.pmi_write_mode       ("normal"), // "normal"|"writethrough"|"readbeforewrite"
	.pmi_family           ("common"), // "common"
	.module_type          ("pmi_ram_dq")  // "pmi_ram_dq"
) ram_0_inst (
	.Data      (TMPI1),  // I:
	.Address   (addr[ADDR_BUS_LEN - 1: 2]),  // I:
	.Clock     (clk),  // I:
	.ClockEn   (cs),  // I:
	.WE        (WRITE1),  // I:
	.Reset     (1'b0),  // I:
	.Q         (TMP[7:0])   // O:
);

pmi_ram_dq
#(
	.pmi_addr_depth       (2 << ADDR_BUS_LEN), // integer       
	.pmi_addr_width       (ADDR_BUS_LEN), // integer       
	.pmi_data_width       (8), // integer
	.pmi_regmode          ("noreg"), // "reg"|"noreg"
	.pmi_gsr              ("enable"), // "enable"|"disable"
	.pmi_resetmode        ("sync"), // "async"|"sync"
	.pmi_optimization     ("speed"), // "area"|"speed"
	.pmi_init_file        (""), // string
	.pmi_init_file_format ("hex"), // "binary"|"hex"
	.pmi_write_mode       ("normal"), // "normal"|"writethrough"|"readbeforewrite"
	.pmi_family           ("common"), // "common"
	.module_type          ("pmi_ram_dq")  // "pmi_ram_dq"
) ram_1_inst (
	.Data      (TMPI2),  // I:
	.Address   (addr[ADDR_BUS_LEN - 1: 2]),  // I:
	.Clock     (clk),  // I:
	.ClockEn   (cs),  // I:
	.WE        (WRITE2),  // I:
	.Reset     (1'b0),  // I:
	.Q         (TMP[15:8])   // O:
);

pmi_ram_dq
#(
	.pmi_addr_depth       (2 << ADDR_BUS_LEN), // integer       
	.pmi_addr_width       (ADDR_BUS_LEN), // integer       
	.pmi_data_width       (8), // integer
	.pmi_regmode          ("noreg"), // "reg"|"noreg"
	.pmi_gsr              ("enable"), // "enable"|"disable"
	.pmi_resetmode        ("sync"), // "async"|"sync"
	.pmi_optimization     ("speed"), // "area"|"speed"
	.pmi_init_file        (""), // string
	.pmi_init_file_format ("hex"), // "binary"|"hex"
	.pmi_write_mode       ("normal"), // "normal"|"writethrough"|"readbeforewrite"
	.pmi_family           ("common"), // "common"
	.module_type          ("pmi_ram_dq")  // "pmi_ram_dq"
) ram_2_inst (
	.Data      (TMPI3),  // I:
	.Address   (addr[ADDR_BUS_LEN - 1: 2]),  // I:
	.Clock     (clk),  // I:
	.ClockEn   (cs),  // I:
	.WE        (WRITE3),  // I:
	.Reset     (1'b0),  // I:
	.Q         (TMP[23:16])   // O:
);

pmi_ram_dq
#(
	.pmi_addr_depth       (2 << ADDR_BUS_LEN), // integer       
	.pmi_addr_width       (ADDR_BUS_LEN), // integer       
	.pmi_data_width       (8), // integer
	.pmi_regmode          ("noreg"), // "reg"|"noreg"
	.pmi_gsr              ("enable"), // "enable"|"disable"
	.pmi_resetmode        ("sync"), // "async"|"sync"
	.pmi_optimization     ("speed"), // "area"|"speed"
	.pmi_init_file        (""), // string
	.pmi_init_file_format ("hex"), // "binary"|"hex"
	.pmi_write_mode       ("normal"), // "normal"|"writethrough"|"readbeforewrite"
	.pmi_family           ("common"), // "common"
	.module_type          ("pmi_ram_dq")  // "pmi_ram_dq"
) ram_3_inst (
	.Data      (TMPI4),  // I:
	.Address   (addr[ADDR_BUS_LEN - 1: 2]),  // I:
	.Clock     (clk),  // I:
	.ClockEn   (cs),  // I:
	.WE        (WRITE4),  // I:
	.Reset     (1'b0),  // I:
	.Q         (TMP[31:24])   // O:
);

end


always @ *
begin
	out = 32'h00000000;
	if(cs)
		out = TMP;
end

end/* !PLATFORM != "iCE40UP"*/
endgenerate

endmodule
/**********************************************************************************************************************/
module rom_dp # (
	parameter PLATFORM = "XILINX",
	parameter EXTENSION_C = "FALSE",
	parameter ADDR_BUS_LEN = 10,
	parameter ROM_PATH = "",
	parameter SYNCHRONOUS_OUTPUT = "TRUE"
	)(
	input clk,
	input [ADDR_BUS_LEN - 1:0]addr_p1,
	input cs_p1,
	output [31:0]out_p1,
	input [ADDR_BUS_LEN - 1:0]addr_p2,
	input cs_p2,
	output reg[31:0]out_p2
    );
 
wire c_instruction = addr_p1[1];
reg c_instruction_delay;
wire [ADDR_BUS_LEN - 1:0]addr_p1_l = addr_p1 + 2;
wire [ADDR_BUS_LEN - 1:0]addr_p1_h = addr_p1;
wire [31:0]TMPO;

 
generate
if(PLATFORM == "XILINX")
begin
(* rom_style="block" *)
reg [31:0]mem[((2**ADDR_BUS_LEN) / 4)-1:0];
(* rom_style="block" *)
reg [15:0]mem_l[((2**ADDR_BUS_LEN) / 4)-1:0];
(* rom_style="block" *)
reg [15:0]mem_h[((2**ADDR_BUS_LEN) / 4)-1:0];

initial begin
if (ROM_PATH != "")
begin
	if(EXTENSION_C == "TRUE")
	begin
		$readmemh({ROM_PATH, "_0.mem"}, mem_l);
		$readmemh({ROM_PATH, "_1.mem"}, mem_h);
	end
	else
	begin
		$readmemh({ROM_PATH, ".mem"}, mem);
	end
end
end

reg [31:0]p2_tmp;
reg [31:0]out_p1_int;
reg [15:0]out_p1_l_int;
reg [15:0]out_p1_h_int;

if(SYNCHRONOUS_OUTPUT == "TRUE")
begin
	always @ (posedge clk)
	begin
		if(EXTENSION_C == "TRUE")
		begin
			out_p1_l_int <= mem_l[addr_p1_l[ADDR_BUS_LEN-1:2]];
			out_p1_h_int <= mem_h[addr_p1_h[ADDR_BUS_LEN-1:2]];
			c_instruction_delay <= c_instruction;
		end
		else
		begin
			out_p1_int <= mem[addr_p1[ADDR_BUS_LEN-1:2]];
		end
	end
end
else
begin
	always @ *
	begin
		if(EXTENSION_C == "TRUE")
		begin
			out_p1_l_int = mem_l[addr_p1_l[ADDR_BUS_LEN-1:2]];
			out_p1_h_int = mem_h[addr_p1_h[ADDR_BUS_LEN-1:2]];
			c_instruction_delay <= c_instruction;
		end
		else
		begin
			out_p1_int = mem[addr_p1[ADDR_BUS_LEN-1:2]];
		end
	end
end

assign out_p1 = cs_p1 ? ((EXTENSION_C == "TRUE") ? (c_instruction_delay ? {out_p1_l_int, out_p1_h_int} : {out_p1_h_int, out_p1_l_int}) : out_p1_int) : 32'h00000000;

reg [31:0]TMP;
reg cs_p2_del;
reg [1:0]addr_p2_del;

if(SYNCHRONOUS_OUTPUT == "TRUE")
begin
	always @ (posedge clk)
	begin
		cs_p2_del <= cs_p2;
		addr_p2_del <= addr_p2[1:0];
		if(EXTENSION_C == "TRUE")
		begin
			TMP <= {mem_h[addr_p2[ADDR_BUS_LEN-1:2]], mem_l[addr_p2[ADDR_BUS_LEN-1:2]]};
		end
		else
		begin
			TMP <= mem[addr_p2[ADDR_BUS_LEN-1:2]];
		end
	end
end
else
begin
	always @ *
	begin
		if(EXTENSION_C == "TRUE")
		begin
			TMP = {mem_h[addr_p2[ADDR_BUS_LEN-1:2]], mem_l[addr_p2[ADDR_BUS_LEN-1:2]]};
		end
		else
		begin
			TMP = mem[addr_p2[ADDR_BUS_LEN-1:2]];
		end
	end
end

always @ *
begin
	out_p2 = 32'h00000000;
	if(cs_p2)
		out_p2 = TMP;
end

end /* PLATFORM != "XILINX" */
else
if(PLATFORM == "LATTICE_ECP5" || PLATFORM == "LATTICE_ECP3" || PLATFORM == "LATTICE_LIFMD" || PLATFORM == "LATTICE_MARCHXO2" || PLATFORM == "LATTICE_MARCHXO3L")
begin/* We use IPExpress to generate the ROM memory with the rom content inside. */



rom rom_inst (
.DataInA(32'b0), 
.DataInB(32'b0), 
.AddressA(addr_p1[ADDR_BUS_LEN-1:2]), 
.AddressB(addr_p2[ADDR_BUS_LEN-1:2]), 
.ClockA(clk), 
.ClockB(clk), 
.ClockEnA(1'b1), 
.ClockEnB(1'b1), 
.WrA(1'b0), 
.WrB(1'b0), 
.ResetA(1'b0), 
.ResetB(1'b0), 
.QA(out_p1), 
.QB(TMPO)
);

always @ *
begin
	out_p2 = 32'h00000000;
	if(cs_p2)
		out_p2 = TMPO;
end

end/* PLATFORM != "LATTICE_ECP5" || PLATFORM != "LATTICE_ECP3" || PLATFORM != "LATTICE_LIFMD" || PLATFORM != "LATTICE_MARCHXO2" || PLATFORM != "LATTICE_MARCHXO3L" */
else if (PLATFORM == "iCE40UP")
begin

pmi_rom 
#(
	.pmi_addr_depth       ((2 << ADDR_BUS_LEN) / 4), // integer       
    .pmi_addr_width       (ADDR_BUS_LEN), // integer       
    .pmi_data_width       (32), // integer       
    .pmi_regmode          ("noreg"), // "reg"|"noreg"     
    .pmi_gsr              ("enable"), // "enable" | "disable"
    .pmi_resetmode        ("sync"), // "async" | "sync"	
    .pmi_init_file        ({ROM_PATH, ".mem"}), // string		
    .pmi_init_file_format ("hex"), // "binary"|"hex"    
	.pmi_family           ("iCE40UP"), // "iCE40UP" | "LIFCL"		
	.module_type          ( )  // string
) rom_inst (
	.Address    (addr_p1[ADDR_BUS_LEN-1:2]),  // I:
	.OutClock   (clk),  // I:
	.OutClockEn (1'b1),  // I:
	.Reset      (1'b0),  // I:
	.Q          (TMPO)   // O:
);
assign out_p1 = cs_p1 ? TMPO : 32'h00000000;
end
endgenerate

endmodule
/************************************************************************************************************************************************/

module rom_dp_cache # (
	parameter PLATFORM = "XILINX",
	parameter ADDR_BUS_LEN = 10,
	parameter ROM_PATH = "",
	parameter BITS_PER_ENTRY_LINE = 128
	)(
	input clk,
	input [ADDR_BUS_LEN - 1:0]addr_p1,
	output [BITS_PER_ENTRY_LINE - 1:0]out_p1,
	input read_en_p1,
	input [ADDR_BUS_LEN - 1:0]addr_p2,
	input cs_p2,
	output reg[31:0]out_p2,
	input read_b_p2,
	input read_h_p2,
	input read_w_p2
    );

(* rom_style="block" *)
reg [BITS_PER_ENTRY_LINE - 1:0]mem[((2**ADDR_BUS_LEN) / (BITS_PER_ENTRY_LINE / 8))-1:0];

initial $readmemh({ROM_PATH, ".mem"}, mem);

reg[BITS_PER_ENTRY_LINE - 1:0]out_p1_int;

//  The following function calculates the address width based on specified data depth
function integer clogb2;
	input integer depth;
	for (clogb2=0; depth>0; clogb2=clogb2+1)
		depth = depth >> 1;
endfunction

always @ (posedge clk)
begin
	out_p1_int <= mem[addr_p1[ADDR_BUS_LEN-1:clogb2((BITS_PER_ENTRY_LINE / 8) - 1)]];
end

reg [BITS_PER_ENTRY_LINE - 1:0]TMP_BIGGER_128;
reg [127:0]TMP;
reg [31:0]TMP32;

always @ (posedge clk)
begin
	TMP_BIGGER_128 <= mem[addr_p2[ADDR_BUS_LEN-1:clogb2((BITS_PER_ENTRY_LINE / 8) - 1)]];
end

always @ *
begin
	if(BITS_PER_ENTRY_LINE == 512)
	begin
		case(addr_p2[5:4])
		2'h0: TMP = TMP_BIGGER_128[127 : 0];
		2'h1: TMP = TMP_BIGGER_128[255 : 128];
		2'h2: TMP = TMP_BIGGER_128[383 : 256];
		2'h3: TMP = TMP_BIGGER_128[511 : 384];
		endcase	
	end
	else if(BITS_PER_ENTRY_LINE == 256)
	begin
		case(addr_p2[4])
		2'h0: TMP = TMP_BIGGER_128[127 : 0];
		2'h1: TMP = TMP_BIGGER_128[255 : 128];
		endcase	
	end
	else
	begin
		TMP = TMP_BIGGER_128;
	end
	case(addr_p2[3:2])
	2'h0: TMP32 = TMP[31:0];
	2'h1: TMP32 = TMP[63:32];
	2'h2: TMP32 = TMP[95:64];
	2'h3: TMP32 = TMP[127:96];
	endcase

	if(read_b_p2 & cs_p2)
	begin
		case(addr_p2[1:0])
		2'h0: out_p2 <= TMP32[7:0];
		2'h1: out_p2 <= TMP32[15:8];
		2'h2: out_p2 <= TMP32[23:16];
		2'h3: out_p2 <= TMP32[31:24];
		endcase
	end
	else if(read_h_p2 & cs_p2)
	begin
		case(addr_p2[1])
		1'h0: out_p2 <= TMP32[15:0];
		1'h1: out_p2 <= TMP32[31:16];
		endcase
	end
	else if(read_w_p2 & cs_p2)
	begin
		out_p2 <= TMP32;
	end
	else
		out_p2 <= 32'h00000000;
end

assign out_p1 = read_en_p1 ? out_p1_int : 32'h0;

endmodule