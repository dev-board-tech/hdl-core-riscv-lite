`define RISC_V_ALU_INST_EXT_I_BEQ_BNE					12'bxx00x1100011
`define RISC_V_ALU_INST_EXT_I_BLT_BGE					12'bxx10x1100011
`define RISC_V_ALU_INST_EXT_I_BLTU_BGEU					12'bxx11x1100011
`define RISC_V_ALU_INST_EXT_I_JAL						12'bxxxxx1101111
`define RISC_V_ALU_INST_EXT_I_JAL7						7'b1101111
`define RISC_V_ALU_INST_EXT_I_JALR						12'bxxxxx1100111
`define RISC_V_ALU_INST_EXT_I_LUI						12'bxxxxx0110111
`define RISC_V_ALU_INST_EXT_I_AUIPC						12'bxxxxx0010111
`define RISC_V_ALU_INST_EXT_I_LB_LH_LW_LBU_LHU			12'bxxxxx0000011
`define RISC_V_ALU_INST_EXT_I_SB_SH_SW					12'bxxxxx0100011
`define RISC_V_ALU_INST_EXT_I_ADDI_ADDIW				12'bxx000001x011
`define RISC_V_ALU_INST_EXT_I_ADD_ADDW					12'b00000011x011
`define RISC_V_ALU_INST_EXT_I_SUB_SUBW					12'b10000011x011
`define RISC_V_ALU_INST_EXT_I_ADD_ADDW_SUB_SUBW			12'bx0000011x011
`define RISC_V_ALU_INST_EXT_I_SLLI_SLLIW				12'b00001001x011
`define RISC_V_ALU_INST_EXT_I_SLL_SLLW					12'b00001011x011
`define RISC_V_ALU_INST_EXT_I_SLL_SLLW_SLLI_SLLIW		12'b000010x1x011
`define RISC_V_ALU_INST_EXT_I_SLTI						12'bxx0100010011
`define RISC_V_ALU_INST_EXT_I_SLT						12'b000100110011
`define RISC_V_ALU_INST_EXT_I_SLTIU						12'bxx0110010011
`define RISC_V_ALU_INST_EXT_I_SLTU						12'b000110110011
`define RISC_V_ALU_INST_EXT_I_SRLI_SRLIW				12'b00101001x011
`define RISC_V_ALU_INST_EXT_I_SRL_SRLW					12'b00101011x011
`define RISC_V_ALU_INST_EXT_I_SRL_SRLW_SRLI_SRLIW		12'b001010x1x011
`define RISC_V_ALU_INST_EXT_I_SRAI_SRAIW				12'b10101001x011
`define RISC_V_ALU_INST_EXT_I_SRA_SRAW					12'b10101011x011
`define RISC_V_ALU_INST_EXT_I_SRA_SRAW_SRAI_SRAIW		12'b101010x1x011
`define RISC_V_ALU_INST_EXT_I_XORI_XOR					12'bxx1000x10011
`define RISC_V_ALU_INST_EXT_I_ORI_OR					12'bxx1100x10011
`define RISC_V_ALU_INST_EXT_I_ANDI_AND					12'bxx1110x10011
`define RISC_V_ALU_INST_EXT_M_MUL						12'b010000110011
`define RISC_V_ALU_INST_EXT_M_MULH						12'b010010110011
`define RISC_V_ALU_INST_EXT_M_MULHSU					12'b010100110011
`define RISC_V_ALU_INST_EXT_M_MULHU						12'b010110110011
`define RISC_V_ALU_INST_EXT_M_DIV						12'b011000110011
`define RISC_V_ALU_INST_EXT_M_DIVU						12'b011010110011
`define RISC_V_ALU_INST_EXT_M_REM						12'b011100110011
`define RISC_V_ALU_INST_EXT_M_REMU						12'b011110110011

`define RISC_V_ALU_INST_EXT_I_LB						3'b000
`define RISC_V_ALU_INST_EXT_I_LH						3'b001
`define RISC_V_ALU_INST_EXT_I_LW						3'b010
`define RISC_V_ALU_INST_EXT_I_LBU						3'b100
`define RISC_V_ALU_INST_EXT_I_LHU						3'b101

`define RISC_V_ALU_INST_EXT_I_SB						2'b00
`define RISC_V_ALU_INST_EXT_I_SH						2'b01
`define RISC_V_ALU_INST_EXT_I_SW						2'b10


`define RISC_V_ALU_INST_EXT_I_R							32'b0x00000xxxxxxxxxxxxxxxxxx0110011
`define RISC_V_ALU_INST_EXT_I_I							32'bxxxxxxxxxxxxxxxxxxxxxxxxx001x011
`define RISC_V_ALU_INST_EXT_I_R_W						32'b0x00000xxxxxxxxxxxxxxxxxx0111011
`define RISC_V_ALU_INST_EXT_M_R							32'b0000001xxxxxxxxxxxxxxxxxx0110011

`define RISC_V_ALU_INST_EXT_I_LOAD						32'bxxxxxxxxxxxxxxxxxxxxxxxxx0000011
`define RISC_V_ALU_INST_EXT_I_STORE						32'bxxxxxxxxxxxxxxxxxxxxxxxxx0100011

`define RISC_V_ALU_INST_BEQ_BNE_BLT_BGE_BLTU_BGEU		32'bxxxxxxxxxxxxxxxxxxxxxxxxx1100011
`define RISC_V_ALU_INST_BEQ_BNE							32'bxxxxxxxxxxxxxxxxx00xxxxxx1100011
`define RISC_V_ALU_INST_BLT_BGE							32'bxxxxxxxxxxxxxxxxx10xxxxxx1100011
`define RISC_V_ALU_INST_BLTU_BGEU						32'bxxxxxxxxxxxxxxxxx11xxxxxx1100011
/* Risc V 32I */
`define RISC_V_ALU_INST_LUI								32'bxxxxxxxxxxxxxxxxxxxxxxxxx0110111
`define RISC_V_ALU_INST_AUIPC							32'bxxxxxxxxxxxxxxxxxxxxxxxxx0010111
`define RISC_V_ALU_INST_JAL								32'bxxxxxxxxxxxxxxxxxxxxxxxxx1101111
`define RISC_V_ALU_INST_JALR							32'bxxxxxxxxxxxxxxxxx000xxxxx1100111
`define RISC_V_ALU_INST_BEQ								32'bxxxxxxxxxxxxxxxxx000xxxxx1100011
`define RISC_V_ALU_INST_BNE								32'bxxxxxxxxxxxxxxxxx001xxxxx1100011
`define RISC_V_ALU_INST_BLT								32'bxxxxxxxxxxxxxxxxx100xxxxx1100011
`define RISC_V_ALU_INST_BGE								32'bxxxxxxxxxxxxxxxxx101xxxxx1100011
`define RISC_V_ALU_INST_BLTU							32'bxxxxxxxxxxxxxxxxx110xxxxx1100011
`define RISC_V_ALU_INST_BGEU							32'bxxxxxxxxxxxxxxxxx111xxxxx1100011
`define RISC_V_ALU_INST_LB								32'bxxxxxxxxxxxxxxxxx000xxxxx0000011
`define RISC_V_ALU_INST_LH								32'bxxxxxxxxxxxxxxxxx001xxxxx0000011
`define RISC_V_ALU_INST_LW								32'bxxxxxxxxxxxxxxxxx010xxxxx0000011
`define RISC_V_ALU_INST_LBU								32'bxxxxxxxxxxxxxxxxx100xxxxx0000011
`define RISC_V_ALU_INST_LHU								32'bxxxxxxxxxxxxxxxxx101xxxxx0000011
`define RISC_V_ALU_INST_SB								32'bxxxxxxxxxxxxxxxxx000xxxxx0100011
`define RISC_V_ALU_INST_SH								32'bxxxxxxxxxxxxxxxxx001xxxxx0100011
`define RISC_V_ALU_INST_SW								32'bxxxxxxxxxxxxxxxxx010xxxxx0100011
`define RISC_V_ALU_INST_ADDI							32'bxxxxxxxxxxxxxxxxxxxxxxxxx0010011
`define RISC_V_ALU_INST_SLTI							32'bxxxxxxxxxxxxxxxxxxxxxxxxx0010011
`define RISC_V_ALU_INST_SLTIU							32'bxxxxxxxxxxxxxxxxxxxxxxxxx0010011
`define RISC_V_ALU_INST_XORI							32'bxxxxxxxxxxxxxxxxxxxxxxxxx0010011
`define RISC_V_ALU_INST_ORI								32'bxxxxxxxxxxxxxxxxxxxxxxxxx0010011
`define RISC_V_ALU_INST_ANDI							32'bxxxxxxxxxxxxxxxxxxxxxxxxx0010011
`define RISC_V_ALU_INST_SLLI							32'b0x00000xxxxxxxxxxxxxxxxxx0010011
`define RISC_V_ALU_INST_SRLI							32'b0x00000xxxxxxxxxxxxxxxxxx0010011
`define RISC_V_ALU_INST_SRAI							32'b0x00000xxxxxxxxxxxxxxxxxx0010011
`define RISC_V_ALU_INST_ADD								32'b0x00000xxxxxxxxxxxxxxxxxx0110011
`define RISC_V_ALU_INST_SUB								32'b0x00000xxxxxxxxxxxxxxxxxx0110011
`define RISC_V_ALU_INST_SLL								32'b0x00000xxxxxxxxxxxxxxxxxx0110011
`define RISC_V_ALU_INST_SLT								32'b0x00000xxxxxxxxxxxxxxxxxx0110011
`define RISC_V_ALU_INST_SLTU							32'b0x00000xxxxxxxxxxxxxxxxxx0110011
`define RISC_V_ALU_INST_XOR								32'b0x00000xxxxxxxxxxxxxxxxxx0110011
`define RISC_V_ALU_INST_SRL								32'b0x00000xxxxxxxxxxxxxxxxxx0110011
`define RISC_V_ALU_INST_SRA								32'b0x00000xxxxxxxxxxxxxxxxxx0110011
`define RISC_V_ALU_INST_OR								32'b0x00000xxxxxxxxxxxxxxxxxx0110011
`define RISC_V_ALU_INST_AND								32'b0x00000xxxxxxxxxxxxxxxxxx0110011
`define RISC_V_ALU_INST_FENCE							32'b0000xxxxxxxx00000000000000001111
`define RISC_V_ALU_INST_FENCE_I							32'b00000000000000000001000000001111
`define RISC_V_ALU_INST_ECALL							32'b00000000000000000000000001110011
`define RISC_V_ALU_INST_EBREAK							32'b00000000000100000000000001110011
`define RISC_V_ALU_INST_LWU								32'bxxxxxxxxxxxxxxxxx110xxxxx0000011
`define RISC_V_ALU_INST_LD								32'bxxxxxxxxxxxxxxxxx011xxxxx0000011
`define RISC_V_ALU_INST_SD								32'bxxxxxxxxxxxxxxxxx011xxxxx0100011
`define RISC_V_ALU_INST_ADDIW							32'bxxxxxxxxxxxxxxxxxxxxxxxxx0011011
`define RISC_V_ALU_INST_SLLIW							32'b0x00000xxxxxxxxxxxxxxxxxx0011011
`define RISC_V_ALU_INST_SRLIW							32'b0x00000xxxxxxxxxxxxxxxxxx0011011
`define RISC_V_ALU_INST_SRAIW							32'b0x00000xxxxxxxxxxxxxxxxxx0011011
`define RISC_V_ALU_INST_ADDW							32'b0x00000xxxxxxxxxxxxxxxxxx0111011
`define RISC_V_ALU_INST_SUBW							32'b0x00000xxxxxxxxxxxxxxxxxx0111011
`define RISC_V_ALU_INST_SLLW							32'b0x00000xxxxxxxxxxxxxxxxxx0111011
`define RISC_V_ALU_INST_SRLW							32'b0x00000xxxxxxxxxxxxxxxxxx0111011
`define RISC_V_ALU_INST_SRAW							32'b0x00000xxxxxxxxxxxxxxxxxx0111011

`define RISC_V_ALU_INST_URET							32'b000000000010xxxxxxxxxxxxx1110011
`define RISC_V_ALU_INST_SRET							32'b000100000010xxxxxxxxxxxxx1110011
`define RISC_V_ALU_INST_HRET							32'b001000000010xxxxxxxxxxxxx1110011
`define RISC_V_ALU_INST_MRET							32'b001100000010xxxxxxxxxxxxx1110011
/*****************************************************************************************/
/* Risc V  Privileged*/
/* Instructions to Access CSRs */
`define RISC_V_ALU_INST_CSRRW							32'bxxxxxxxxxxxxxxxxx001xxxxx1110011
`define RISC_V_ALU_INST_CSRRS							32'bxxxxxxxxxxxxxxxxx010xxxxx1110011
`define RISC_V_ALU_INST_CSRRC							32'bxxxxxxxxxxxxxxxxx011xxxxx1110011
`define RISC_V_ALU_INST_CSRRWWI							32'bxxxxxxxxxxxxxxxxx101xxxxx1110011
`define RISC_V_ALU_INST_CSRRWSI							32'bxxxxxxxxxxxxxxxxx110xxxxx1110011
`define RISC_V_ALU_INST_CSRRWCI							32'bxxxxxxxxxxxxxxxxx111xxxxx1110011
/* Instructions to Change Privilege Level */
`define RISC_V_ALU_INST_ECALL							32'b00000000000000000000000001110011
`define RISC_V_ALU_INST_EBREAK							32'b00000000000100000000000001110011
`define RISC_V_ALU_INST_ERET							32'b00010000000000000000000001110011
/* Trap-Redirection Instructions */
`define RISC_V_ALU_INST_MRTS							32'b00110000010100000000000001110011
`define RISC_V_ALU_INST_MRTH							32'b00110000011000000000000001110011
`define RISC_V_ALU_INST_HRTS							32'b00100000010100000000000001110011
/* Interrupt management */
`define RISC_V_ALU_INST_WFI								32'b00010000001000000000000001110011
/* Memory-Management Instructions */
`define RISC_V_ALU_INST_SFENCE_VM						32'b000100000001xxxxx000000001110011
/*****************************************************************************************/
`define REG_MIE_MTIE_bp									7
`define REG_MIE_MSIE_bp									3

`define REG_MIE_ADDR									0x304;//Machine interrupt-enable register.
`define REG_MTIMECMP_ADDR								0x321;//Machine wall-clock timer compare value.
`define REG_MTIME_ADDR									0x701;//Machine wall-clock time.
`define REG_MTIMEH_ADDR									0x741;//Upper 32 bits of mtime, RV32I only.
`define REG_MEPC_ADDR									0x341;//Machine exception program counter.
`define REG_MCAUSE_ADDR									0x342;//Machine trap cause.
`define REG_MBADADDR_ADDR								0x343;//Machine bad address.







