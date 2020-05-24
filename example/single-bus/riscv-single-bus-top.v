/*
 * This IP is the Lite RISC-V RV32I single bus top implementation.
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

/*
 Run at 57 Mhz on Artix7
 */

`timescale 1ns / 1ps

`undef ADDR_BUS_WIDTH
`define ADDR_BUS_WIDTH			16

`undef ROM_ADDR_WIDTH
`define ROM_ADDR_WIDTH			15

`undef RAM_ADDR_WIDTH
`define RAM_ADDR_WIDTH			13

`undef C_EXTENSION
`define C_EXTENSION	"FALSE"
`undef SYNCHRONOUS_ROM
`define SYNCHRONOUS_ROM "TRUE"
`undef SYNCHRONOUS_RAM
`define SYNCHRONOUS_RAM "TRUE"

module riscv_single_bus_top(
	input rst,
	input clk,
	output [7:0]LED,
	input [7:0]SW
    );

wire [`ADDR_BUS_WIDTH - 1:0]pgm_addr;
wire [31:0]pgm_out;
wire pgm_write_w;
wire pgm_write_h;
wire pgm_write_b;
wire [31:0]pgm_in;

wire pgm_en = pgm_addr[`ADDR_BUS_WIDTH - 1];
wire pgm_data_en = pgm_addr[`ADDR_BUS_WIDTH - 1: `ADDR_BUS_WIDTH - 2] == 2'b01;
wire pgm_io_en = pgm_addr[`ADDR_BUS_WIDTH - 1:`ADDR_BUS_WIDTH - 3] == 3'b001;

wire core_clk;//= clk;
wire clkfb;
wire pll_locked;

/* For manual step by stem clock debugging. */
//always @ (posedge clk) core_clk <= SW[0]; 

reg pll_locked_delayed;
always @ (posedge core_clk) pll_locked_delayed <= pll_locked;

PLLE2_BASE #(
	.BANDWIDTH("OPTIMIZED"),  // OPTIMIZED, HIGH, LOW
	.CLKFBOUT_MULT(16),        // Multiply value for all CLKOUT, (2-64)
	.CLKFBOUT_PHASE(0.0),     // Phase offset in degrees of CLKFB, (-360.000-360.000).
	.CLKIN1_PERIOD(10.0),      // Input clock period in ns to ps resolution (i.e. 33.333 is 30 MHz).
	// CLKOUT0_DIVIDE - CLKOUT5_DIVIDE: Divide amount for each CLKOUT (1-128)
	.CLKOUT0_DIVIDE(28),
	.CLKOUT1_DIVIDE(1),
	.CLKOUT2_DIVIDE(1),
	.CLKOUT3_DIVIDE(1),
	.CLKOUT4_DIVIDE(1),
	.CLKOUT5_DIVIDE(1),
	// CLKOUT0_DUTY_CYCLE - CLKOUT5_DUTY_CYCLE: Duty cycle for each CLKOUT (0.001-0.999).
	.CLKOUT0_DUTY_CYCLE(0.5),
	.CLKOUT1_DUTY_CYCLE(0.5),
	.CLKOUT2_DUTY_CYCLE(0.5),
	.CLKOUT3_DUTY_CYCLE(0.5),
	.CLKOUT4_DUTY_CYCLE(0.5),
	.CLKOUT5_DUTY_CYCLE(0.5),
	// CLKOUT0_PHASE - CLKOUT5_PHASE: Phase offset for each CLKOUT (-360.000-360.000).
	.CLKOUT0_PHASE(0.0),
	.CLKOUT1_PHASE(0.0),
	.CLKOUT2_PHASE(0.0),
	.CLKOUT3_PHASE(0.0),
	.CLKOUT4_PHASE(0.0),
	.CLKOUT5_PHASE(0.0),
	.DIVCLK_DIVIDE(1),        // Master division value, (1-56)
	.REF_JITTER1(0.0),        // Reference input jitter in UI, (0.000-0.999).
	.STARTUP_WAIT("TRUE")    // Delay DONE until PLL Locks, ("TRUE"/"FALSE")
)
PLLE2_BASE_inst (
	// Clock Outputs: 1-bit (each) output: User configurable clock outputs
	.CLKOUT0(core_clk),   // 1-bit output: CLKOUT0
	.CLKOUT1(),   // 1-bit output: CLKOUT1
	.CLKOUT2(),   // 1-bit output: CLKOUT2
	.CLKOUT3(),   // 1-bit output: CLKOUT3
	.CLKOUT4(),   // 1-bit output: CLKOUT4
	.CLKOUT5(),   // 1-bit output: CLKOUT5
	// Feedback Clocks: 1-bit (each) output: Clock feedback ports
	.CLKFBOUT(clkfb), // 1-bit output: Feedback clock
	.LOCKED(pll_locked),     // 1-bit output: LOCK
	.CLKIN1(clk),     // 1-bit input: Input clock
	// Control Ports: 1-bit (each) input: PLL control ports
	.PWRDWN(),     // 1-bit input: Power-down
	.RST(~rst),           // 1-bit input: Reset
	// Feedback Clocks: 1-bit (each) input: Clock feedback ports
	.CLKFBIN(clkfb)    // 1-bit input: Feedback clock
);

wire [31:0]pioA_bus_out;
atmega_pio # (
	.PLATFORM("XILINX"),
	.BUS_ADDR_DATA_LEN(13),
	.PORT_WIDTH(32),
	.PORT_OUT_ADDR('h00),
	.PORT_CLEAR_ADDR('h04),
	.PORT_SET_ADDR('h08),
	.DDR_ADDR('h0c),
	.PIN_ADDR('h10),
	.PINMASK(32'h000000FF),
	.PULLUP_MASK(32'h0),
	.PULLDN_MASK(32'h0),
	.INVERSE_MASK(32'hFFFFFFFF),
	.OUT_ENABLED_MASK(32'h000000FF)
	)
pio_A_inst(
	.rst(~pll_locked_delayed | ~rst),
	.clk(core_clk),

	.addr(pgm_addr[12:0]),
	.wr(pgm_write_w & pgm_io_en),
	.rd(pgm_io_en),
	.bus_in(pgm_out),
	.bus_out(pioA_bus_out),

	.io_in(SW),
	.io_out(LED),
	.pio_out_io_connect()
	);

wire [31:0]rom_bus_out;
/* Use the RAM module instead of ROM to allocate all memory and to selfwrite from a bootloader if is the case. */
ram # (
	.ADDR_BUS_LEN(`ROM_ADDR_WIDTH),
	.RAM_PATH("riscvLiteTstApp"),
	.SYNCHRONOUS_OUTPUT(`SYNCHRONOUS_ROM)
	)
rom_inst(
	.clk(core_clk),
	.addr(pgm_addr[`ROM_ADDR_WIDTH - 1:0]),
	.cs(pgm_en),
	.out(rom_bus_out),
	.in(pgm_out),
	.write_w(pgm_write_w),
	.write_h(pgm_write_h),
	.write_b(pgm_write_b)
);

wire [31:0]ram_bus_out;
ram # (
	.ADDR_BUS_LEN(`RAM_ADDR_WIDTH),
	.SYNCHRONOUS_OUTPUT(`SYNCHRONOUS_RAM)
	)
ram_inst(
	.clk(core_clk),
	.addr(pgm_addr[`RAM_ADDR_WIDTH - 1:0]),
	.cs(pgm_data_en),
	.out(ram_bus_out),
	.in(pgm_out),
	.write_w(pgm_write_w),
	.write_h(pgm_write_h),
	.write_b(pgm_write_b)
);

bus_dmux #(
	.NR_OF_BUSSES_IN(3)
	)
bus_dmux_inst(
	.bus_in({ 
	pioA_bus_out, 
	rom_bus_out,
	ram_bus_out
	}),
	.bus_out(pgm_in)
	);

risc_v_lite_single_bus #
	(
	.PLATFORM("XILINX"),
	.ADDR_BUS_WIDTH(`ADDR_BUS_WIDTH),
	.RESET_VECTOR(32'h8000),
	.SYNCHRONOUS_ROM(`SYNCHRONOUS_ROM),
	.SYNCHRONOUS_RAM(`SYNCHRONOUS_RAM),
	.EXTENSION_M("FALSE"),
	.EXTENSION_MDIV("FALSE"),
	.EXTENSION_C(`C_EXTENSION),
	.WATCHDOG_CNT_WIDTH(0),
	.VECTOR_INT_TABLE_SIZE(0)
)risc_v_lite_inst(
	.rst(~pll_locked_delayed | ~rst),
	.clk(core_clk),
	.core_stall(1'b0),
	.pgm_addr(pgm_addr),
	.pgm_out(pgm_out),
	.pgm_write_w(pgm_write_w),
	.pgm_write_h(pgm_write_h),
	.pgm_write_b(pgm_write_b),
	.pgm_in(pgm_in),
	.int_sig(),
	.int_rst(),
	.int_en(),
	.debug()
	);

endmodule
