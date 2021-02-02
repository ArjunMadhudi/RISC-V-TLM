/*!
 \file extension_base.h
 \brief Base class for ISA extensions
 \author Màrius Montón
 \date May 2020
 */
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef INC_EXTENSION_BASE_H_
#define INC_EXTENSION_BASE_H_

#include "systemc"

#include "Instruction.h"
#include "Registers.h"
#include "Log.h"
#include "MemoryInterface.h"

#define EXCEPTION_CAUSE_INSTRUCTION_MISALIGN  0
#define EXCEPTION_CAUSE_INSTRUCTION_ACCESS    1
#define EXCEPTION_CAUSE_ILLEGAL_INSTRUCTION   2
#define EXCEPTION_CAUSE_BREAKPOINT            3
#define EXCEPTION_CAUSE_LOAD_ADDR_MISALIGN    4
#define EXCEPTION_CAUSE_LOAD_ACCESS_FAULT     5

template<typename T>
class extension_base {

public:


	/* pure virtual functions */
	virtual int32_t opcode() const = 0;

	virtual int32_t get_rd() const {
	    return m_instr.range(11, 7);
	}

	virtual void set_rd(int32_t value) {
    m_instr.range(11, 7) = value;
	}

	virtual int32_t get_rs1() const {
    return m_instr.range(19, 15);
	}

	virtual void set_rs1(int32_t value) {
	  m_instr.range(19, 15) = value;
	}

	virtual int32_t get_rs2() const {
	  return m_instr.range(24, 20);
	}

	virtual void set_rs2(int32_t value) {
    m_instr.range(24, 20) = value;
	}

	virtual int32_t get_funct3() const {
    return m_instr.range(14, 12);
	}

	virtual void set_funct3(int32_t value) {
	  m_instr.range(14, 12) = value;
	}

	extension_base(sc_dt::sc_uint<32> const instr,
		Registers<T> *register_bank, MemoryInterface *mem_interface) :
		m_instr(instr), regs(register_bank), mem_intf(mem_interface) {
		perf = Performance::getInstance();
		log = Log::getInstance();
	}


	virtual ~extension_base() =default;


void setInstr(uint32_t p_instr) {
	m_instr = sc_dt::sc_uint<32>(p_instr);
}


inline virtual void dump() const {
	std::cout << std::hex << "0x" << m_instr << std::dec << std::endl;
}


void RaiseException(uint32_t cause, uint32_t inst) {
	uint32_t new_pc, current_pc, m_cause;

	current_pc = regs->getPC();
	m_cause = regs->getCSR(CSR_MSTATUS);
	m_cause |= cause;

	new_pc = regs->getCSR(CSR_MTVEC);

	regs->setCSR(CSR_MEPC, current_pc);

	if (cause == EXCEPTION_CAUSE_ILLEGAL_INSTRUCTION) {
		regs->setCSR(CSR_MTVAL, inst);
	} else {
		regs->setCSR(CSR_MTVAL, current_pc);
	}

	regs->setCSR(CSR_MCAUSE, cause);
	regs->setCSR(CSR_MSTATUS, m_cause);

	regs->setPC(new_pc);

	log->SC_log(Log::ERROR) << "Exception! new PC 0x" << std::hex << new_pc
			<< std::endl;

	regs->dump();
	std::cout << "Simulation time " << sc_core::sc_time_stamp() << std::endl;
	perf->dump();

	sc_core::sc_stop();
}


bool NOP() {

	log->SC_log(Log::INFO) << "NOP" << "\n";

	return true;
}

protected:
	sc_dt::sc_uint<32> m_instr;
	Registers<T> *regs;
	Performance *perf;
	Log *log;
	MemoryInterface *mem_intf;
};

#endif /* INC_EXTENSION_BASE_H_ */
