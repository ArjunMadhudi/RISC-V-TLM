/*!
 \file BASE_ISA.h
 \brief RISC-V ISA implementation
 \author Màrius Montón
 \date August 2018
 */
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef Execute_H
#define Execute_H

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include "systemc"
#include "tlm.h"
#include "tlm_utils/simple_initiator_socket.h"

#include "memory.h"
#include "MemoryInterface.h"
#include "Instruction.h"
#include "C_extension.h"
#include "M_extension.h"
#include "A_extension.h"
#include "Registers.h"
#include "Log.h"

enum {
	LUI = 0b0110111,
	AUIPC = 0b0010111,
	JAL = 0b1101111,
	JALR = 0b1100111,

	BEQ = 0b1100011,
	BEQ_F = 0b000,
	BNE_F = 0b001,
	BLT_F = 0b100,
	BGE_F = 0b101,
	BLTU_F = 0b110,
	BGEU_F = 0b111,

	LB = 0b0000011,
	LB_F = 0b000,
	LH_F = 0b001,
	LW_F = 0b010,
	LBU_F = 0b100,
	LHU_F = 0b101,

	SB = 0b0100011,
	SB_F = 0b000,
	SH_F = 0b001,
	SW_F = 0b010,

	ADDI = 0b0010011,
	ADDI_F = 0b000,
	SLTI_F = 0b010,
	SLTIU_F = 0b011,
	XORI_F = 0b100,
	ORI_F = 0b110,
	ANDI_F = 0b111,
	SLLI_F = 0b001,
	SRLI_F = 0b101,
	SRLI_F7 = 0b0000000,
	SRAI_F7 = 0b0100000,

	ADD = 0b0110011,
	ADD_F = 0b000,
	SUB_F = 0b000,
	ADD_F7 = 0b0000000,
	SUB_F7 = 0b0100000,

	SLL_F = 0b001,
	SLT_F = 0b010,
	SLTU_F = 0b011,
	XOR_F = 0b100,
	SRL_F = 0b101,
	SRA_F = 0b101,
	SRL_F7 = 0b0000000,
	SRA_F7 = 0b0100000,
	OR_F = 0b110,
	AND_F = 0b111,

	FENCE = 0b0001111,
	ECALL = 0b1110011,
	ECALL_F = 0b000000000000,
	EBREAK_F = 0b000000000001,
	URET_F = 0b000000000010,
	SRET_F = 0b000100000010,
	MRET_F = 0b001100000010,
	WFI_F = 0b000100000101,
	SFENCE_F = 0b0001001,

	ECALL_F3 = 0b000,
	CSRRW = 0b001,
	CSRRS = 0b010,
	CSRRC = 0b011,
	CSRRWI = 0b101,
	CSRRSI = 0b110,
	CSRRCI = 0b111,
};

typedef enum {
	OP_LUI,
	OP_AUIPC,
	OP_JAL,
	OP_JALR,

	OP_BEQ,
	OP_BNE,
	OP_BLT,
	OP_BGE,
	OP_BLTU,
	OP_BGEU,

	OP_LB,
	OP_LH,
	OP_LW,
	OP_LBU,
	OP_LHU,

	OP_SB,
	OP_SH,
	OP_SW,

	OP_ADDI,
	OP_SLTI,
	OP_SLTIU,
	OP_XORI,
	OP_ORI,
	OP_ANDI,
	OP_SLLI,
	OP_SRLI,
	OP_SRAI,

	OP_ADD,
	OP_SUB,
	OP_SLL,
	OP_SLT,
	OP_SLTU,
	OP_XOR,
	OP_SRL,
	OP_SRA,
	OP_OR,
	OP_AND,

	OP_FENCE,
	OP_ECALL,
	OP_EBREAK,

	OP_CSRRW,
	OP_CSRRS,
	OP_CSRRC,
	OP_CSRRWI,
	OP_CSRRSI,
	OP_CSRRCI,

	OP_URET,
	OP_SRET,
	OP_MRET,
	OP_WFI,
	OP_SFENCE,

	OP_ERROR
} opCodes;

/**
 * @brief Risc_V execute module
 */
template<typename T>
class BASE_ISA: public extension_base<T> {
public:

	/**
	 * @brief Constructor, same as base class
	 */
	using extension_base<T>::extension_base;
	using extension_base<T>::m_instr;
	using extension_base<T>::get_rd;
	using extension_base<T>::get_rs1;
	using extension_base<T>::get_rs2;
	using extension_base<T>::regs;
	using extension_base<T>::mem_intf;
	using extension_base<T>::perf;
	using extension_base<T>::log;
	using extension_base<T>::setInstr;
	using extension_base<T>::NOP;
	using extension_base<T>::get_funct3;
	using extension_base<T>::RaiseException;

	/**
	 * @brief Access to funct7 field
	 * @return funct7 field
	 */
	inline int32_t get_funct7() const {
		return m_instr.range(31, 25);
	}

	/**
	 * @brief Sets func7 field
	 * @param value desired func7 value
	 */
	inline void set_func7(int32_t value) {
		m_instr.range(31, 25) = value;
	}

	/**
	 * @brief Gets immediate field value for I-type
	 * @return immediate_I field
	 */
	inline int32_t get_imm_I() const {
		int32_t aux = 0;

		aux = m_instr.range(31, 20);

		/* sign extension (optimize) */
		if (m_instr[31] == 1) {
			aux |= (0b11111111111111111111) << 12;
		}

		return aux;
	}

	/**
	 * @brief Sets immediate field for I-type
	 * @param value desired I value
	 */
	inline void set_imm_I(int32_t value) {
		m_instr.range(31, 20) = value;
	}

	/**
	 * @brief Gets immediate field value for S-type
	 * @return immediate_S field
	 */
	inline int32_t get_imm_S() const {
		int32_t aux = 0;

		aux = m_instr.range(31, 25) << 5;
		aux |= m_instr.range(11, 7);

		if (m_instr[31] == 1) {
			aux |= (0b11111111111111111111) << 12;
		}

		return aux;
	}

	/**
	 * @brief Gets immediate field value for U-type
	 * @return immediate_U field
	 */
	inline int32_t get_imm_U() const {
		return m_instr.range(31, 12);
	}

	/**
	 * @brief Sets immediate field for U-type
	 * @param value desired U value
	 */
	inline void set_imm_U(int32_t value) {
		m_instr.range(31, 12) = (value << 12);
	}

	/**
	 * @brief Gets immediate field value for B-type
	 * @return immediate_B field
	 */
	inline int32_t get_imm_B() const {
		int32_t aux = 0;

		aux |= m_instr[7] << 11;
		aux |= m_instr.range(30, 25) << 5;
		aux |= m_instr[31] << 12;
		aux |= m_instr.range(11, 8) << 1;

		if (m_instr[31] == 1) {
			aux |= (0b11111111111111111111) << 12;
		}

		return aux;
	}

	/**
	 * @brief Sets immediate field for B-type
	 * @param value desired B value
	 */
	inline void set_imm_B(int32_t value) {
		sc_dt::sc_uint<32> aux = value;

		m_instr[31] = aux[12];
		m_instr.range(30, 25) = aux.range(10, 5);
		m_instr.range(11, 7) = aux.range(4, 1);
		m_instr[6] = aux[11];
	}

	/**
	 * @brief Gets immediate field value for J-type
	 * @return immediate_J field
	 */
	inline int32_t get_imm_J() const {
		int32_t aux = 0;

		aux = m_instr[31] << 20;
		aux |= m_instr.range(19, 12) << 12;
		aux |= m_instr[20] << 11;
		aux |= m_instr.range(30, 21) << 1;

		/* bit extension (better way to do that?) */
		if (m_instr[31] == 1) {
			aux |= (0b111111111111) << 20;
		}

		return aux;
	}

	/**
	 * @brief Sets immediate field for J-type
	 * @param value desired J value
	 */
	inline void set_imm_J(int32_t value) {
		sc_dt::sc_uint<32> aux = (value << 20);

		m_instr[31] = aux[20];
		m_instr.range(30, 21) = aux.range(10, 1);
		m_instr[20] = aux[11];
		m_instr.range(19, 12) = aux.range(19, 12);
	}

	/**
	 * @brief Returns shamt field for Shifts instructions
	 * @return value corresponding to inst(25:20)
	 */
	inline int32_t get_shamt() const {
		return m_instr.range(25, 20);
	}

	/**
	 * @brief Returns CSR field for CSR instructions
	 * @return value corresponding to instr(31:20)
	 */
	inline int32_t get_csr() const {
		int32_t aux = 0;

		aux = m_instr.range(31, 20);

		return aux;
	}

	/**
	 * @brief Access to opcode field
	 * @return return opcode field
	 */
	inline int32_t opcode() const override
	{
		return m_instr.range(6, 0);
	}

	enum {
		LUI = 0b0110111,
		AUIPC = 0b0010111,
		JAL = 0b1101111,
		JALR = 0b1100111,

		BEQ = 0b1100011,
		BEQ_F = 0b000,
		BNE_F = 0b001,
		BLT_F = 0b100,
		BGE_F = 0b101,
		BLTU_F = 0b110,
		BGEU_F = 0b111,

		LB = 0b0000011,
		LB_F = 0b000,
		LH_F = 0b001,
		LW_F = 0b010,
		LBU_F = 0b100,
		LHU_F = 0b101,

		SB = 0b0100011,
		SB_F = 0b000,
		SH_F = 0b001,
		SW_F = 0b010,

		ADDI = 0b0010011,
		ADDI_F = 0b000,
		SLTI_F = 0b010,
		SLTIU_F = 0b011,
		XORI_F = 0b100,
		ORI_F = 0b110,
		ANDI_F = 0b111,
		SLLI_F = 0b001,
		SRLI_F = 0b101,
		SRLI_F7 = 0b0000000,
		SRAI_F7 = 0b0100000,

		ADD = 0b0110011,
		ADD_F = 0b000,
		SUB_F = 0b000,
		ADD_F7 = 0b0000000,
		SUB_F7 = 0b0100000,

		SLL_F = 0b001,
		SLT_F = 0b010,
		SLTU_F = 0b011,
		XOR_F = 0b100,
		SRL_F = 0b101,
		SRA_F = 0b101,
		SRL_F7 = 0b0000000,
		SRA_F7 = 0b0100000,
		OR_F = 0b110,
		AND_F = 0b111,

		FENCE = 0b0001111,
		ECALL = 0b1110011,
		ECALL_F = 0b000000000000,
		EBREAK_F = 0b000000000001,
		URET_F = 0b000000000010,
		SRET_F = 0b000100000010,
		MRET_F = 0b001100000010,
		WFI_F = 0b000100000101,
		SFENCE_F = 0b0001001,

		ECALL_F3 = 0b000,
		CSRRW = 0b001,
		CSRRS = 0b010,
		CSRRC = 0b011,
		CSRRWI = 0b101,
		CSRRSI = 0b110,
		CSRRCI = 0b111,
	};

	bool Exec_LUI() const {
		T rd;
		uint32_t imm = 0;

		rd = get_rd();
		imm = get_imm_U() << 12;
		regs->setValue(rd, imm);

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "LUI x" << std::dec << rd << " <- 0x"
					<< std::hex << imm << "\n";
		}

		return true;
	}

	bool Exec_AUIPC() const {
		T rd;
		uint32_t imm = 0;
		T new_pc;

		rd = get_rd();
		imm = get_imm_U() << 12;
		new_pc = regs->getPC() + imm;

		regs->setValue(rd, new_pc);

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "AUIPC x" << std::dec << rd << " <- 0x"
					<< std::hex << imm << " + PC (0x" << new_pc << ")" << "\n";
		}

		return true;
	}

	bool Exec_JAL() const {
		int32_t mem_addr = 0;
		T rd;
		T new_pc, old_pc;

		rd = get_rd();
		mem_addr = get_imm_J();
		old_pc = regs->getPC();
		new_pc = old_pc + mem_addr;

		regs->setPC(new_pc);

		old_pc = old_pc + 4;
		regs->setValue(rd, old_pc);

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "JAL: x" << std::dec << rd << " <- 0x"
					<< std::hex << old_pc << std::dec << ". PC + 0x" << std::hex
					<< mem_addr << " -> PC (0x" << new_pc << ")" << "\n";
		}

		return true;
	}

	bool Exec_JALR() const {
		uint32_t mem_addr = 0;
		T rd, rs1;
		T new_pc, old_pc;

		rd = get_rd();
		rs1 = get_rs1();
		mem_addr = get_imm_I();

		old_pc = regs->getPC();
		regs->setValue(rd, old_pc + 4);

		new_pc = (regs->getValue(rs1) + mem_addr) & 0xFFFFFFFE;
		regs->setPC(new_pc);

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "JALR: x" << std::dec << rd << " <- 0x"
					<< std::hex << old_pc + 4 << " PC <- 0x" << new_pc << "\n";
		}
		return true;
	}

	bool Exec_BEQ() const {
		T rs1, rs2;
		T new_pc = 0;

		rs1 = get_rs1();
		rs2 = get_rs2();

		if (regs->getValue(rs1) == regs->getValue(rs2)) {
			new_pc = regs->getPC() + get_imm_B();
			regs->setPC(new_pc);
		} else {
			regs->incPC();
			new_pc = regs->getPC();
		}

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "BEQ x" << std::dec << rs1 << "(0x"
					<< std::hex << regs->getValue(rs1) << ") == x" << std::dec
					<< rs2 << "(0x" << std::hex << regs->getValue(rs2)
					<< ")? -> PC (0x" << std::hex << new_pc << ")" << std::dec
					<< "\n";
		}

		return true;
	}

	bool Exec_BNE() const {
		T rs1, rs2;
		T new_pc = 0;
		T val1, val2;

		rs1 = get_rs1();
		rs2 = get_rs2();

		val1 = regs->getValue(rs1);
		val2 = regs->getValue(rs2);

		if (val1 != val2) {
			new_pc = regs->getPC() + get_imm_B();
			regs->setPC(new_pc);
		} else {
			regs->incPC();
			new_pc = regs->getPC();
		}

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "BNE: x" << std::dec << rs1 << "(0x"
					<< std::hex << val1 << ") == x" << std::dec << rs2 << "(0x"
					<< std::hex << val2 << ")? -> PC (0x" << std::hex << new_pc
					<< ")" << std::dec << "\n";
		}

		return true;
	}

	bool Exec_BLT() const {
		T rs1, rs2;
		T new_pc = 0;

		rs1 = get_rs1();
		rs2 = get_rs2();

		if ((int32_t) regs->getValue(rs1) < (int32_t) regs->getValue(rs2)) {
			new_pc = regs->getPC() + get_imm_B();
			regs->setPC(new_pc);
		} else {
			regs->incPC();
		}

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "BLT x" << std::dec << rs1 << "(0x"
					<< std::hex << (int32_t) regs->getValue(rs1) << ") < x"
					<< std::dec << rs2 << "(0x" << std::hex
					<< (int32_t) regs->getValue(rs2) << ")? -> PC (0x"
					<< std::hex << new_pc << ")" << std::dec << "\n";
		}

		return true;
	}

	bool Exec_BGE() const {
		T rs1, rs2;
		T new_pc = 0;

		rs1 = get_rs1();
		rs2 = get_rs2();

		if ((int32_t) regs->getValue(rs1) >= (int32_t) regs->getValue(rs2)) {
			new_pc = regs->getPC() + get_imm_B();
			regs->setPC(new_pc);
		} else {
			regs->incPC();
		}

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "BGE x" << std::dec << rs1 << "(0x"
					<< std::hex << (int32_t) regs->getValue(rs1) << ") > x"
					<< std::dec << rs2 << "(0x" << std::hex
					<< (int32_t) regs->getValue(rs2) << ")? -> PC (0x"
					<< std::hex << new_pc << ")" << std::dec << "\n";
		}

		return true;
	}

	bool Exec_BLTU() const {
		T rs1, rs2;
		T new_pc = 0;

		rs1 = get_rs1();
		rs2 = get_rs2();

		if ((uint32_t) regs->getValue(rs1) < (uint32_t) regs->getValue(rs2)) {
			new_pc = regs->getPC() + get_imm_B();
			regs->setPC(new_pc);
		} else {
			regs->incPC();
			new_pc = regs->getPC();
		}

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "BLTU x" << std::dec << rs1 << "(0x"
					<< std::hex << regs->getValue(rs1) << ") < x" << std::dec
					<< rs2 << "(0x" << std::hex << regs->getValue(rs2)
					<< ")? -> PC (0x" << std::hex << new_pc << ")" << std::dec
					<< "\n";
		}

		return true;
	}

	bool Exec_BGEU() const {
		T rs1, rs2;
		T new_pc = 0;

		rs1 = get_rs1();
		rs2 = get_rs2();

		if ((uint32_t) regs->getValue(rs1) >= (uint32_t) regs->getValue(rs2)) {
			new_pc = regs->getPC() + get_imm_B();
			regs->setPC(new_pc);
		} else {
			regs->incPC();
		}

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "BGEU x" << std::dec << rs1 << "(0x"
					<< std::hex << regs->getValue(rs1) << ") > x" << std::dec
					<< rs2 << "(0x" << std::hex << regs->getValue(rs2)
					<< ")? -> PC (0x" << std::hex << new_pc << ")" << std::dec
					<< "\n";
		}

		return true;
	}

	bool Exec_LB() const {
		T mem_addr = 0;
		T rd, rs1;
		int32_t imm = 0;
		int8_t data;

		rd = get_rd();
		rs1 = get_rs1();
		imm = get_imm_I();

		mem_addr = imm + regs->getValue(rs1);
		data = mem_intf->readDataMem(mem_addr, 1);
		perf->dataMemoryRead();
		regs->setValue(rd, data);

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "LB: x" << rs1 << " + " << imm << " (@0x"
					<< std::hex << mem_addr << std::dec << ") -> x" << rd
					<< "\n";
		}

		return true;
	}

	bool Exec_LH() const {
		T mem_addr = 0;
		T rd, rs1;
		int32_t imm = 0;
		int16_t data;

		rd = get_rd();
		rs1 = get_rs1();
		imm = get_imm_I();

		mem_addr = imm + regs->getValue(rs1);
		data = mem_intf->readDataMem(mem_addr, 2);
		perf->dataMemoryRead();
		regs->setValue(rd, data);

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "LH: x" << rs1 << " + " << imm << " (@0x"
					<< std::hex << mem_addr << std::dec << ") -> x" << rd
					<< "\n";
		}

		return true;
	}

	/* template specialization */
	bool
	Exec_LW() const;

	bool Exec_LBU() const {
		T mem_addr = 0;
		T rd, rs1;
		int32_t imm = 0;
		uint8_t data;

		rd = get_rd();
		rs1 = get_rs1();
		imm = get_imm_I();

		mem_addr = imm + regs->getValue(rs1);
		data = mem_intf->readDataMem(mem_addr, 1);
		perf->dataMemoryRead();
		regs->setValue(rd, data);

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "LBU: x" << rs1 << " + " << imm << " (@0x"
					<< std::hex << mem_addr << std::dec << ") -> x" << rd
					<< "\n";
		}
		return true;
	}

	bool Exec_LHU() const {
		T mem_addr = 0;
		T rd, rs1;
		int32_t imm = 0;
		uint16_t data;

		rd = get_rd();
		rs1 = get_rs1();
		imm = get_imm_I();

		mem_addr = imm + regs->getValue(rs1);
		data = mem_intf->readDataMem(mem_addr, 2);
		perf->dataMemoryRead();

		regs->setValue(rd, data);

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "LHU: x" << std::dec << rs1 << " + "
					<< imm << " (@0x" << std::hex << mem_addr << std::dec
					<< ") -> x" << rd << "(0x" << std::hex << data << ")"
					<< "\n";
		}

		return true;
	}

	/* template specialization */
	bool
	Exec_LD() const;

	bool Exec_SB() const {
		T mem_addr = 0;
		T rs1, rs2;
		int32_t imm = 0;
		T data;

		rs1 = get_rs1();
		rs2 = get_rs2();
		imm = get_imm_S();

		mem_addr = imm + regs->getValue(rs1);
		data = regs->getValue(rs2);

		mem_intf->writeDataMem(mem_addr, data, 1);
		perf->dataMemoryWrite();

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "SB: x" << std::dec << rs2 << " -> x"
					<< rs1 << " + 0x" << std::hex << imm << " (@0x" << std::hex
					<< mem_addr << std::dec << ")" << "\n";
		}

		return true;
	}

	bool Exec_SH() const {
		T mem_addr = 0;
		T rs1, rs2;
		int32_t imm = 0;
		T data;

		rs1 = get_rs1();
		rs2 = get_rs2();
		imm = get_imm_S();

		mem_addr = imm + regs->getValue(rs1);
		data = regs->getValue(rs2);

		mem_intf->writeDataMem(mem_addr, data, 2);
		perf->dataMemoryWrite();

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "SH: x" << std::dec << rs2 << " -> x"
					<< rs1 << " + 0x" << std::hex << imm << " (@0x" << std::hex
					<< mem_addr << std::dec << ")" << "\n";
		}

		return true;
	}

	bool Exec_SW() const {
		T mem_addr = 0;
		T rs1, rs2;
		int32_t imm = 0;
		T data;

		rs1 = get_rs1();
		rs2 = get_rs2();
		imm = get_imm_S();

		mem_addr = imm + regs->getValue(rs1);
		data = regs->getValue(rs2);

		mem_intf->writeDataMem(mem_addr, data, 4);
		perf->dataMemoryWrite();

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "SW: x" << std::dec << rs2 << "(0x"
					<< std::hex << data << ") -> x" << std::dec << rs1
					<< " + 0x" << std::hex << imm << " (@0x" << std::hex
					<< mem_addr << std::dec << ")" << "\n";
		}
		return true;
	}

	/*
	 * Present only in RV64, template specialization
	 */
	template<uint64_t>
	bool Exec_SD() const {
		T mem_addr = 0;
		T rs1, rs2;
		int32_t imm = 0;
		T data;

		rs1 = get_rs1();
		rs2 = get_rs2();
		imm = get_imm_S();

		mem_addr = imm + regs->getValue(rs1);
		data = regs->getValue(rs2);

		mem_intf->writeDataMem(mem_addr, data, 8);
		perf->dataMemoryWrite();

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "SD: x" << std::dec << rs2 << "(0x"
					<< std::hex << data << ") -> x" << std::dec << rs1
					<< " + 0x" << std::hex << imm << " (@0x" << std::hex
					<< mem_addr << std::dec << ")" << "\n";
		}
		return true;
	}

	bool Exec_ADDI() const {
		T rd, rs1;
		int32_t imm = 0;
		T calc;

		rd = get_rd();
		rs1 = get_rs1();
		imm = get_imm_I();

		calc = regs->getValue(rs1) + imm;
		regs->setValue(rd, calc);

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "ADDI: x" << std::dec << rs1 << " + "
					<< imm << " -> x" << std::dec << rd << "(0x" << std::hex
					<< calc << ")" << "\n";
		}

		return true;
	}

	bool Exec_SLTI() const {
		T rd, rs1;
		int32_t imm;

		rd = get_rd();
		rs1 = get_rs1();
		imm = get_imm_I();

		if (static_cast<int32_t>(regs->getValue(rs1)) < imm) {
			regs->setValue(rd, 1);
			log->SC_log(Log::INFO) << "SLTI: x" << rs1 << " < " << imm << " => "
					<< "1 -> x" << rd << "\n";
		} else {
			regs->setValue(rd, 0);
			log->SC_log(Log::INFO) << "SLTI: x" << rs1 << " < " << imm << " => "
					<< "0 -> x" << rd << "\n";
		}

		return true;
	}

	bool Exec_SLTIU() const {
		T rd, rs1;
		int32_t imm;

		rd = get_rd();
		rs1 = get_rs1();
		imm = get_imm_I();

		if ((uint32_t) regs->getValue(rs1) < (uint32_t) imm) {
			regs->setValue(rd, 1);
			log->SC_log(Log::INFO) << "SLTIU: x" << rs1 << " < " << imm
					<< " => " << "1 -> x" << rd << "\n";
		} else {
			regs->setValue(rd, 0);
			log->SC_log(Log::INFO) << "SLTIU: x" << rs1 << " < " << imm
					<< " => " << "0 -> x" << rd << "\n";
		}

		return true;
	}

	bool Exec_XORI() const {
		T rd, rs1;
		int32_t imm;
		T calc;

		rd = get_rd();
		rs1 = get_rs1();
		imm = get_imm_I();

		calc = regs->getValue(rs1) ^ imm;
		regs->setValue(rd, calc);

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "XORI: x" << rs1 << " XOR " << imm
					<< "-> x" << rd << "\n";
		}

		return true;
	}

	bool Exec_ORI() const {
		T rd, rs1;
		int32_t imm;
		T calc;

		rd = get_rd();
		rs1 = get_rs1();
		imm = get_imm_I();

		calc = regs->getValue(rs1) | imm;
		regs->setValue(rd, calc);

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "ORI: x" << rs1 << " OR " << imm << "-> x"
					<< rd << "\n";
		}

		return true;
	}

	bool Exec_ANDI() const {
		T rd, rs1;
		uint32_t imm;
		T calc;
		T aux;

		rd = get_rd();
		rs1 = get_rs1();
		imm = get_imm_I();

		aux = regs->getValue(rs1);
		calc = aux & imm;
		regs->setValue(rd, calc);

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "ANDI: x" << rs1 << "(0x" << std::hex
					<< aux << ") AND 0x" << imm << " -> x" << std::dec << rd
					<< "(0x" << std::hex << calc << ")" << "\n";
		}

		return true;
	}

	bool Exec_SLLI() {
		T rd, rs1, rs2;
		uint32_t shift;
		T calc;

		rd = get_rd();
		rs1 = get_rs1();
		rs2 = get_shamt();

		if (rs2 >= 0x20) {
			std::cout << "ILEGAL INSTRUCTION, shamt[5] != 0" << "\n";
			RaiseException(EXCEPTION_CAUSE_ILLEGAL_INSTRUCTION, m_instr);

			return false;
		}

		shift = rs2 & 0x1F;

		calc = ((uint32_t) regs->getValue(rs1)) << shift;
		regs->setValue(rd, calc);

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "SLLI: x" << std::dec << rs1 << " << "
					<< shift << " -> x" << rd << "(0x" << std::hex << calc
					<< ")" << "\n";
		}

		return true;
	}

	bool Exec_SRLI() const {
		T rd, rs1, rs2;
		uint32_t shift;
		T calc;

		rd = get_rd();
		rs1 = get_rs1();
		rs2 = get_rs2();

		shift = rs2 & 0x1F;

		calc = ((uint32_t) regs->getValue(rs1)) >> shift;
		regs->setValue(rd, calc);

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "SRLI: x" << std::dec << rs1 << " >> "
					<< shift << " -> x" << rd << "\n";
		}

		return true;
	}

	bool Exec_SRAI() const {
		T rd, rs1, rs2;
		uint32_t shift;
		T calc;

		rd = get_rd();
		rs1 = get_rs1();
		rs2 = get_rs2();

		shift = rs2 & 0x1F;

		calc = regs->getValue(rs1) >> shift;
		regs->setValue(rd, calc);

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "SRAI: x" << std::dec << rs1 << " >> "
					<< shift << " -> x" << rd << "\n";
		}

		return true;
	}

	bool Exec_ADD() const {
		T rd, rs1, rs2;
		T calc;
		rd = get_rd();
		rs1 = get_rs1();
		rs2 = get_rs2();

		calc = regs->getValue(rs1) + regs->getValue(rs2);

		regs->setValue(rd, calc);

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "ADD: x" << std::dec << rs1 << " + x"
					<< rs2 << " -> x" << rd << std::hex << "(0x" << calc << ")"
					<< "\n";
		}

		return true;
	}

	bool Exec_SUB() const {
		T rd, rs1, rs2;
		T calc;
		rd = get_rd();
		rs1 = get_rs1();
		rs2 = get_rs2();

		calc = regs->getValue(rs1) - regs->getValue(rs2);
		regs->setValue(rd, calc);

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "SUB: x" << std::dec << rs1 << " - x" << rs2
					<< " -> x" << rd << std::hex << "(0x" << calc << ")" << "\n";
		}

		return true;
	}

	bool Exec_SLL() const {
		T rd, rs1, rs2;
		uint32_t shift;
		T calc;

		rd = get_rd();
		rs1 = get_rs1();
		rs2 = get_rs2();

		shift = regs->getValue(rs2) & 0x1F;

		calc = ((uint32_t) regs->getValue(rs1)) << shift;
		regs->setValue(rd, calc);

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "SLL: x" << rs1 << " << " << shift
					<< " -> x" << rd << "\n";
		}

		return true;
	}

	bool Exec_SLT() const {
		T rd, rs1, rs2;

		rd = get_rd();
		rs1 = get_rs1();
		rs2 = get_rs2();

		if (regs->getValue(rs1) < regs->getValue(rs2)) {
			regs->setValue(rd, 1);
			log->SC_log(Log::INFO) << "SLT: x" << rs1 << " < x" << rs2 << " => "
					<< "1 -> x" << rd << "\n";
		} else {
			regs->setValue(rd, 0);
			log->SC_log(Log::INFO) << "SLT: x" << rs1 << " < x" << rs2 << " => "
					<< "0 -> x" << rd << "\n";
		}

		return true;
	}

	bool Exec_SLTU() const {
		T rd, rs1, rs2;

		rd = get_rd();
		rs1 = get_rs1();
		rs2 = get_rs2();

		if ((uint32_t) regs->getValue(rs1) < (uint32_t) regs->getValue(rs2)) {
			regs->setValue(rd, 1);
			log->SC_log(Log::INFO) << "SLTU: x" << rs1 << " < x" << rs2
					<< " => " << "1 -> x" << rd << "\n";
		} else {
			regs->setValue(rd, 0);
			log->SC_log(Log::INFO) << "SLTU: x" << rs1 << " < x" << rs2
					<< " => " << "0 -> x" << rd << "\n";
		}

		return true;
	}

	bool Exec_XOR() const {
		T rd, rs1, rs2;
		T calc;

		rd = get_rd();
		rs1 = get_rs1();
		rs2 = get_rs2();

		calc = regs->getValue(rs1) ^ regs->getValue(rs2);
		regs->setValue(rd, calc);

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "XOR: x" << rs1 << " XOR x" << rs2
					<< "-> x" << rd << "\n";
		}

		return true;
	}

	bool Exec_SRL() const {
		T rd, rs1, rs2;
		uint32_t shift;
		T calc;

		rd = get_rd();
		rs1 = get_rs1();
		rs2 = get_rs2();

		shift = regs->getValue(rs2) & 0x1F;

		calc = ((uint32_t) regs->getValue(rs1)) >> shift;
		regs->setValue(rd, calc);

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "SRL: x" << rs1 << " >> " << shift
					<< " -> x" << rd << "\n";
		}

		return true;
	}

	bool Exec_SRA() const {
		T rd, rs1, rs2;
		uint32_t shift;
		T calc;

		rd = get_rd();
		rs1 = get_rs1();
		rs2 = get_rs2();

		shift = regs->getValue(rs2) & 0x1F;

		calc = regs->getValue(rs1) >> shift;
		regs->setValue(rd, calc);

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "SRA: x" << rs1 << " >> " << shift
					<< " -> x" << rd << "\n";
		}

		return true;
	}

	bool Exec_OR() const {
		T rd, rs1, rs2;
		T calc;

		rd = get_rd();
		rs1 = get_rs1();
		rs2 = get_rs2();

		calc = regs->getValue(rs1) | regs->getValue(rs2);
		regs->setValue(rd, calc);

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "OR: x" << rs1 << " OR x" << rs2 << "-> x"
					<< rd << "\n";
		}

		return true;
	}

	bool Exec_AND() const {
		T rd, rs1, rs2;
		T calc;

		rd = get_rd();
		rs1 = get_rs1();
		rs2 = get_rs2();

		calc = regs->getValue(rs1) & regs->getValue(rs2);
		regs->setValue(rd, calc);

		if (log->getLogLevel() >= Log::INFO) {
			log->SC_log(Log::INFO) << "AND: x" << rs1 << " AND x" << rs2
					<< "-> x" << rd << "\n";
		}

		return true;
	}

	bool Exec_FENCE() const {
		log->SC_log(Log::INFO) << "FENCE" << "\n";

		return true;
	}

	bool Exec_ECALL() const {

		log->SC_log(Log::INFO) << "ECALL" << "\n";
		std::cout << "\n" << "ECALL Instruction called, stopping simulation"
				<< "\n";
		regs->dump();
		std::cout << "Simulation time " << sc_core::sc_time_stamp() << "\n";
		perf->dump();

		uint32_t gp_value = regs->getValue(Registers<uint32_t>::gp);
		if (gp_value == 1) {
			std::cout << "GP value is 1, test result is OK" << "\n";
		} else {
			std::cout << "GP value is " << gp_value << "\n";
		}

		sc_core::sc_stop();

		return true;
	}

	bool Exec_EBREAK() {

		log->SC_log(Log::INFO) << "EBREAK" << "\n";
		std::cout << "\n" << "EBRAK  Instruction called, dumping information"
				<< "\n";
		regs->dump();
		std::cout << "Simulation time " << sc_core::sc_time_stamp() << "\n";
		perf->dump();

		sc_core::sc_stop();

		return true;
	}

	bool Exec_CSRRW() const {
		int rd, rs1;
		int csr;
		uint32_t aux;

		rd = get_rd();
		rs1 = get_rs1();
		csr = get_csr();

		/* These operations must be atomical */
		if (rd != 0) {
			aux = regs->getCSR(csr);
			regs->setValue(rd, aux);
		}

		aux = regs->getValue(rs1);
		regs->setCSR(csr, aux);

		log->SC_log(Log::INFO) << std::hex << "CSRRW: CSR #" << csr << " -> x"
				<< std::dec << rd << ". x" << rs1 << "-> CSR #" << std::hex
				<< csr << " (0x" << aux << ")" << "\n";

		return true;
	}

	bool Exec_CSRRS() const {
		int rd, rs1;
		int csr;
		uint32_t bitmask, aux, aux2;

		rd = get_rd();
		rs1 = get_rs1();
		csr = get_csr();

		if (rd == 0) {
			log->SC_log(Log::INFO) << "CSRRS with rd1 == 0, doing nothing."
					<< "\n";
			return false;
		}

		/* These operations must be atomical */
		aux = regs->getCSR(csr);
		bitmask = regs->getValue(rs1);

		regs->setValue(rd, aux);

		aux2 = aux | bitmask;
		regs->setCSR(csr, aux2);

		log->SC_log(Log::INFO) << "CSRRS: CSR #" << csr << "(0x" << std::hex
				<< aux << ") -> x" << std::dec << rd << ". x" << rs1
				<< " & CSR #" << csr << " <- 0x" << std::hex << aux2 << "\n";

		return true;
	}

	bool Exec_CSRRC() const {
		int rd, rs1;
		int csr;
		uint32_t bitmask, aux, aux2;

		rd = get_rd();
		rs1 = get_rs1();
		csr = get_csr();

		if (rd == 0) {
			log->SC_log(Log::INFO) << "CSRRC with rd1 == 0, doing nothing."
					<< "\n";
			return true;
		}

		/* These operations must be atomical */
		aux = regs->getCSR(csr);
		bitmask = regs->getValue(rs1);

		regs->setValue(rd, aux);

		aux2 = aux & ~bitmask;
		regs->setCSR(csr, aux2);

		log->SC_log(Log::INFO) << "CSRRC: CSR #" << csr << "(0x" << std::hex
				<< aux << ") -> x" << std::dec << rd << ". x" << rs1
				<< " & CSR #" << csr << " <- 0x" << std::hex << aux2 << "\n";

		return true;
	}

	bool Exec_CSRRWI() const {
		int rd, rs1;
		int csr;
		uint32_t aux;

		rd = get_rd();
		rs1 = get_rs1();
		csr = get_csr();

		/* These operations must be atomical */
		if (rd != 0) {
			aux = regs->getCSR(csr);
			regs->setValue(rd, aux);
		}
		aux = rs1;
		regs->setCSR(csr, aux);

		log->SC_log(Log::INFO) << "CSRRWI: CSR #" << csr << " -> x" << rd
				<< ". x" << rs1 << "-> CSR #" << csr << "\n";

		return true;
	}

	bool Exec_CSRRSI() const {
		int rd, rs1;
		int csr;
		uint32_t bitmask, aux;

		rd = get_rd();
		rs1 = get_rs1();
		csr = get_csr();

		if (rs1 == 0) {
			return true;
		}

		/* These operations must be atomical */
		aux = regs->getCSR(csr);
		regs->setValue(rd, aux);

		bitmask = rs1;
		aux = aux | bitmask;
		regs->setCSR(csr, aux);

		log->SC_log(Log::INFO) << "CSRRSI: CSR #" << csr << " -> x" << rd
				<< ". x" << rs1 << " & CSR #" << csr << "(0x" << std::hex << aux
				<< ")" << "\n";

		return true;
	}

	bool Exec_CSRRCI() const {
		int rd, rs1;
		int csr;
		uint32_t bitmask, aux;

		rd = get_rd();
		rs1 = get_rs1();
		csr = get_csr();

		if (rs1 == 0) {
			return true;
		}

		/* These operations must be atomical */
		aux = regs->getCSR(csr);
		regs->setValue(rd, aux);

		bitmask = rs1;
		aux = aux & ~bitmask;
		regs->setCSR(csr, aux);

		log->SC_log(Log::INFO) << "CSRRCI: CSR #" << csr << " -> x" << rd
				<< ". x" << rs1 << " & CSR #" << csr << "(0x" << std::hex << aux
				<< ")" << "\n";

		return true;
	}

	/*********************** Privileged Instructions ******************************/

	bool Exec_MRET() const {
		uint32_t new_pc = 0;

		new_pc = regs->getCSR(CSR_MEPC);
		regs->setPC(new_pc);

		log->SC_log(Log::INFO) << "MRET: PC <- 0x" << std::hex << new_pc
				<< "\n";

		// update mstatus
		uint32_t csr_temp;
		csr_temp = regs->getCSR(CSR_MSTATUS);
		if (csr_temp & MSTATUS_MPIE) {
			csr_temp |= MSTATUS_MIE;
		}
		csr_temp |= MSTATUS_MPIE;
		regs->setCSR(CSR_MSTATUS, csr_temp);

		return true;
	}

	bool Exec_SRET() const {
		uint32_t new_pc = 0;

		new_pc = regs->getCSR(CSR_SEPC);
		regs->setPC(new_pc);

		log->SC_log(Log::INFO) << "SRET: PC <- 0x" << std::hex << new_pc
				<< "\n";

		return true;
	}

	bool Exec_WFI() const {
		log->SC_log(Log::INFO) << "WFI" << "\n";

		return true;
	}

	bool Exec_SFENCE() const {
		log->SC_log(Log::INFO) << "SFENCE" << "\n";

		return true;
	}

	bool process_instruction(Instruction *inst) {
		bool PC_not_affected = true;

		setInstr(inst->getInstr());

		switch (decode()) {
		case OP_LUI:
			Exec_LUI();
			break;
		case OP_AUIPC:
			Exec_AUIPC();
			break;
		case OP_JAL:
			Exec_JAL();
			PC_not_affected = false;
			break;
		case OP_JALR:
			Exec_JALR();
			PC_not_affected = false;
			break;
		case OP_BEQ:
			Exec_BEQ();
			PC_not_affected = false;
			break;
		case OP_BNE:
			Exec_BNE();
			PC_not_affected = false;
			break;
		case OP_BLT:
			Exec_BLT();
			PC_not_affected = false;
			break;
		case OP_BGE:
			Exec_BGE();
			PC_not_affected = false;
			break;
		case OP_BLTU:
			Exec_BLTU();
			PC_not_affected = false;
			break;
		case OP_BGEU:
			Exec_BGEU();
			PC_not_affected = false;
			break;
		case OP_LB:
			Exec_LB();
			break;
		case OP_LH:
			Exec_LH();
			break;
		case OP_LW:
			Exec_LW();
			break;
		case OP_LBU:
			Exec_LBU();
			break;
		case OP_LHU:
			Exec_LHU();
			break;
		case OP_SB:
			Exec_SB();
			break;
		case OP_SH:
			Exec_SH();
			break;
		case OP_SW:
			Exec_SW();
			break;
		case OP_ADDI:
			Exec_ADDI();
			break;
		case OP_SLTI:
			Exec_SLTI();
			break;
		case OP_SLTIU:
			Exec_SLTIU();
			break;
		case OP_XORI:
			Exec_XORI();
			break;
		case OP_ORI:
			Exec_ORI();
			break;
		case OP_ANDI:
			Exec_ANDI();
			break;
		case OP_SLLI:
			PC_not_affected = Exec_SLLI();
			break;
		case OP_SRLI:
			Exec_SRLI();
			break;
		case OP_SRAI:
			Exec_SRAI();
			break;
		case OP_ADD:
			Exec_ADD();
			break;
		case OP_SUB:
			Exec_SUB();
			break;
		case OP_SLL:
			Exec_SLL();
			break;
		case OP_SLT:
			Exec_SLT();
			break;
		case OP_SLTU:
			Exec_SLTU();
			break;
		case OP_XOR:
			Exec_XOR();
			break;
		case OP_SRL:
			Exec_SRL();
			break;
		case OP_SRA:
			Exec_SRA();
			break;
		case OP_OR:
			Exec_OR();
			break;
		case OP_AND:
			Exec_AND();
			break;
		case OP_FENCE:
			Exec_FENCE();
			break;
		case OP_ECALL:
			Exec_ECALL();
			break;
		case OP_EBREAK:
			Exec_EBREAK();
			break;
		case OP_CSRRW:
			Exec_CSRRW();
			break;
		case OP_CSRRS:
			Exec_CSRRS();
			break;
		case OP_CSRRC:
			Exec_CSRRC();
			break;
		case OP_CSRRWI:
			Exec_CSRRWI();
			break;
		case OP_CSRRSI:
			Exec_CSRRSI();
			break;
		case OP_CSRRCI:
			Exec_CSRRCI();
			break;
		case OP_MRET:
			Exec_MRET();
			PC_not_affected = false;
			break;
		case OP_SRET:
			Exec_SRET();
			PC_not_affected = false;
			break;
		case OP_WFI:
			Exec_WFI();
			break;
		case OP_SFENCE:
			Exec_SFENCE();
			break;
		[[unlikely]] default:
			std::cout << "Wrong instruction" << "\n";
			inst->dump();
			NOP();
			//sc_stop();
			break;
		}

		return PC_not_affected;
	}

	opCodes decode() {
		switch (opcode()) {
		case LUI:
			return OP_LUI;
		case AUIPC:
			return OP_AUIPC;
		case JAL:
			return OP_JAL;
		case JALR:
			return OP_JALR;
		case BEQ:
			switch (get_funct3()) {
			case BEQ_F:
				return OP_BEQ;
			case BNE_F:
				return OP_BNE;
			case BLT_F:
				return OP_BLT;
			case BGE_F:
				return OP_BGE;
			case BLTU_F:
				return OP_BLTU;
			case BGEU_F:
				return OP_BGEU;
			}
			return OP_ERROR;
		case LB:
			switch (get_funct3()) {
			case LB_F:
				return OP_LB;
			case LH_F:
				return OP_LH;
			case LW_F:
				return OP_LW;
			case LBU_F:
				return OP_LBU;
			case LHU_F:
				return OP_LHU;
			}
			return OP_ERROR;
		case SB:
			switch (get_funct3()) {
			case SB_F:
				return OP_SB;
			case SH_F:
				return OP_SH;
			case SW_F:
				return OP_SW;
			}
			return OP_ERROR;
		case ADDI:
			switch (get_funct3()) {
			case ADDI_F:
				return OP_ADDI;
			case SLTI_F:
				return OP_SLTI;
			case SLTIU_F:
				return OP_SLTIU;
			case XORI_F:
				return OP_XORI;
			case ORI_F:
				return OP_ORI;
			case ANDI_F:
				return OP_ANDI;
			case SLLI_F:
				return OP_SLLI;
			case SRLI_F:
				switch (get_funct7()) {
				case SRLI_F7:
					return OP_SRLI;
				case SRAI_F7:
					return OP_SRAI;
				}
				return OP_ERROR;
			}
			return OP_ERROR;
		case ADD: {
			switch (get_funct3()) {
			case ADD_F:
				switch (get_funct7()) {
				case ADD_F7:
					return OP_ADD;
				case SUB_F7:
					return OP_SUB;
				default:
					return OP_ADD;
				}
				break;
			case SLL_F:
				return OP_SLL;
			case SLT_F:
				return OP_SLT;
			case SLTU_F:
				return OP_SLTU;
			case XOR_F:
				return OP_XOR;
			case SRL_F:
				switch (get_funct7()) {
				case SRL_F7:
					return OP_SRL;
				case SRA_F7:
					return OP_SRA;
				default:
					return OP_ERROR;
				}
			case OR_F:
				return OP_OR;
			case AND_F:
				return OP_AND;
			default:
				return OP_ERROR;
			}
		} /* ADD */
		case FENCE:
			return OP_FENCE;
		case ECALL: {
			switch (get_funct3()) {
			case ECALL_F3:
				switch (get_csr()) {
				case ECALL_F:
					return OP_ECALL;
				case EBREAK_F:
					return OP_EBREAK;
				case URET_F:
					return OP_URET;
				case SRET_F:
					return OP_SRET;
				case MRET_F:
					return OP_MRET;
				case WFI_F:
					return OP_WFI;
				case SFENCE_F:
					return OP_SFENCE;
				}
				if (m_instr.range(31, 25) == 0b0001001) {
					return OP_SFENCE;
				}
				break;
			case CSRRW:
				return OP_CSRRW;
				break;
			case CSRRS:
				return OP_CSRRS;
				break;
			case CSRRC:
				return OP_CSRRC;
				break;
			case CSRRWI:
				return OP_CSRRWI;
				break;
			case CSRRSI:
				return OP_CSRRSI;
				break;
			case CSRRCI:
				return OP_CSRRCI;
				break;
			}
		}
			break;
		default:
			return OP_ERROR;
		}

		return OP_ERROR;
	}

};

/* template specialization for RV32*/
template<>
bool BASE_ISA<uint32_t>::Exec_LW() const {
	uint32_t mem_addr = 0;
	uint32_t rd, rs1;
	int32_t imm = 0;
	uint32_t data;

	rd = get_rd();
	rs1 = get_rs1();
	imm = get_imm_I();

	mem_addr = imm + regs->getValue(rs1);
	data = mem_intf->readDataMem(mem_addr, 4);
	perf->dataMemoryRead();

	regs->setValue(rd, data);

	if (log->getLogLevel() >= Log::INFO) {
		log->SC_log(Log::INFO) << std::dec << "LW: x" << rs1 << "(0x"
				<< std::hex << regs->getValue(rs1) << ") + " << std::dec << imm
				<< " (@0x" << std::hex << mem_addr << std::dec << ") -> x" << rd
				<< std::hex << " (0x" << data << ")" << "\n";
	}
	return true;
}

/* template specialization for RV64*/
template<>
bool BASE_ISA<uint64_t>::Exec_LW() const {
	uint64_t mem_addr = 0;
	uint64_t rd, rs1;
	int32_t imm = 0;
	uint64_t data;

	rd = get_rd();
	rs1 = get_rs1();
	imm = get_imm_I();

	mem_addr = imm + regs->getValue(rs1);
	data = mem_intf->readDataMem(mem_addr, 4);
	perf->dataMemoryRead();

	/* sign extension from 32 bit to 64bit */
	data = (data << 32) >> 32;
	regs->setValue(rd, data);

	if (log->getLogLevel() >= Log::INFO) {
		log->SC_log(Log::INFO) << std::dec << "LW: x" << rs1 << "(0x"
				<< std::hex << regs->getValue(rs1) << ") + " << std::dec << imm
				<< " (@0x" << std::hex << mem_addr << std::dec << ") -> x" << rd
				<< std::hex << " (0x" << data << ")" << "\n";
	}
	return true;
}

/*
 * Present only in RV64, template specialization
 */
template<>
bool BASE_ISA<uint64_t>::Exec_LD() const {
	uint64_t mem_addr = 0;
	uint64_t rd, rs1;
	int32_t imm = 0;
	uint64_t data;

	rd = get_rd();
	rs1 = get_rs1();
	imm = get_imm_I();

	mem_addr = imm + regs->getValue(rs1);
	data = mem_intf->readDataMem(mem_addr, 8);
	perf->dataMemoryRead();
	regs->setValue(rd, data);

	if (log->getLogLevel() >= Log::INFO) {
		log->SC_log(Log::INFO) << std::dec << "LW: x" << rs1 << "(0x"
				<< std::hex << regs->getValue(rs1) << ") + " << std::dec << imm
				<< " (@0x" << std::hex << mem_addr << std::dec << ") -> x" << rd
				<< std::hex << " (0x" << data << ")" << "\n";
	}

	return true;
}

#endif
