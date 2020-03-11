/*
 * This IP is the simplifyed IO headerfile definition.
 * 
 * Copyright (C) 2017  Iulian Gheorghiu (morgoth@devboard.tech)
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
--------------------------------------------------------------------------
System registers in IO space
--------------------------------------------------------------------------
*/

`define REG_MSTATUS					('h300 << 2)	//Machine status register.
`define REG_MIE						('h304 << 2)	//Machine interrupt-enable register.
`define REG_MSCRATCH				('h340 << 2)	//Scratch register for machine trap handlers.
`define REG_MEPC					('h341 << 2)	//Machine exception program counter.
`define REG_MCAUSE					('h342 << 2)	//Machine trap cause.
`define REG_MBADADDR				('h343 << 2)	//Machine bad address.

`define REG_MSTATUS_MIE_bp			3
`define REG_MSTATUS_HIE_bp			2
`define REG_MSTATUS_SIE_bp			1
`define REG_MSTATUS_UIE_bp			0
