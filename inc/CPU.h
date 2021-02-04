/*!
 \file CPU.h
 \brief Main CPU class
 \author Màrius Montón
 \date August 2018
 */
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef CPU_BASE_H
#define CPU_BASE_H

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include "systemc"

#include "tlm.h"
#include "tlm_utils/simple_initiator_socket.h"
#include "tlm_utils/tlm_quantumkeeper.h"

#include "memory.h"
#include "MemoryInterface.h"
#include "BASE_ISA.h"
#include "Registers.h"
#include "Log.h"
#include "Instruction.h"
#include "C_extension.h"
#include "M_extension.h"
#include "A_extension.h"

/**
 * @brief ISC_V CPU model
 * @param name name of the module
 */
template<typename T>
class CPU: sc_core::sc_module {
public:

	/**
	 * @brief Instruction Memory bus socket
	 * @param trans transction to perfoem
	 * @param delay time to annotate
	 */
	tlm_utils::simple_initiator_socket<CPU> instr_bus;

	/**
	 * @brief IRQ line socket
	 * @param trans transction to perform (empty)
	 * @param delay time to annotate
	 */
	tlm_utils::simple_target_socket<CPU> irq_line_socket;

	SC_HAS_PROCESS(CPU);

	/**
	 * @brief Constructor
	 * @param name Module name
	 * @param PC   Program Counter initialize value
	 */
	CPU(sc_core::sc_module_name const name, uint32_t PC) :
			sc_module(name), instr_bus("instr_bus"), default_time(10,
					sc_core::SC_NS) {
		register_bank = new Registers<T>();
		mem_intf = new MemoryInterface();

		perf = Performance::getInstance();
		log = Log::getInstance();

		register_bank->setPC(PC);
		register_bank->setValue(Registers<T>::sp, (0x10000000 / 4) - 1);

		irq_line_socket.register_b_transport(this, &CPU::call_interrupt);
		interrupt = false;

		int_cause = 0;
		irq_already_down = false;

		dmi_ptr_valid = false;
		instr_bus.register_invalidate_direct_mem_ptr(this,
				&CPU::invalidate_direct_mem_ptr);

		inst = new Instruction(0);
		exec = new BASE_ISA<T>(0, register_bank, mem_intf);
		c_inst = new C_extension<T>(0, register_bank, mem_intf);
		m_inst = new M_extension<T>(0, register_bank, mem_intf);
		a_inst = new A_extension<T>(0, register_bank, mem_intf);

		m_qk = new tlm_utils::tlm_quantumkeeper();

		SC_THREAD(CPU_thread);
	}

	/**
	 * @brief Destructor
	 */

	~CPU() {
		delete register_bank;
		delete mem_intf;
		delete inst;
		delete exec;
		delete c_inst;
		delete m_inst;
		delete a_inst;
		delete m_qk;
		delete log;
	}

	/**
	 *
	 * @brief Process and triggers IRQ if all conditions met
	 * @return true if IRQ is triggered, false otherwise
	 */
	bool cpu_process_IRQ() {
		uint32_t csr_temp;
		bool ret_value = false;

		if (interrupt == true) {
			csr_temp = register_bank->getCSR(CSR_MSTATUS);
			if ((csr_temp & MSTATUS_MIE) == 0) {
				log->SC_log(Log::DEBUG) << "interrupt delayed" << std::endl;
				return ret_value;
			}

			csr_temp = register_bank->getCSR(CSR_MIP);

			if ((csr_temp & MIP_MEIP) == 0) {
				csr_temp |= MIP_MEIP;  // MEIP bit in MIP register (11th bit)
				register_bank->setCSR(CSR_MIP, csr_temp);
				log->SC_log(Log::DEBUG) << "Interrupt!" << std::endl;

				/* updated MEPC register */
				uint32_t old_pc = register_bank->getPC();
				register_bank->setCSR(CSR_MEPC, old_pc);
				log->SC_log(Log::INFO) << "Old PC Value 0x" << std::hex
						<< old_pc << std::endl;

				/* update MCAUSE register */
				register_bank->setCSR(CSR_MCAUSE, 0x80000000);

				/* set new PC address */
				uint32_t new_pc = register_bank->getCSR(CSR_MTVEC);
				//new_pc = new_pc & 0xFFFFFFFC; // last two bits always to 0
				log->SC_log(Log::DEBUG) << "NEW PC Value 0x" << std::hex
						<< new_pc << std::endl;
				register_bank->setPC(new_pc);

				ret_value = true;
				interrupt = false;
				irq_already_down = false;
			}
		} else {
			if (irq_already_down == false) {
				csr_temp = register_bank->getCSR(CSR_MIP);
				csr_temp &= ~MIP_MEIP;
				register_bank->setCSR(CSR_MIP, csr_temp);
				irq_already_down = true;
			}
		}

		return ret_value;
	}

	/**
	 * main thread for CPU simulation
	 * @brief CPU mai thread
	 */
	void CPU_thread(void) {

		tlm::tlm_generic_payload *trans = new tlm::tlm_generic_payload;
		uint32_t INSTR;
		sc_core::sc_time delay = sc_core::SC_ZERO_TIME;
		bool PC_not_affected = false;
		bool incPCby2 = false;
		tlm::tlm_dmi dmi_data;
		unsigned char *dmi_ptr = nullptr;

		trans->set_command(tlm::TLM_READ_COMMAND);
		trans->set_data_ptr(reinterpret_cast<unsigned char*>(&INSTR));
		trans->set_data_length(4);
		trans->set_streaming_width(4); // = data_length to indicate no streaming
		trans->set_byte_enable_ptr(0); // 0 indicates unused
		trans->set_dmi_allowed(false); // Mandatory initial value
		trans->set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

		m_qk->reset();

		while (1) {
			/* Get new PC value */
			if (dmi_ptr_valid == true) {
				/* if memory_offset at Memory module is set, this won't work */
				memcpy(&INSTR, dmi_ptr + register_bank->getPC(), 4);
			} else {
				trans->set_address(register_bank->getPC());
				instr_bus->b_transport(*trans, delay);

				if (trans->is_response_error()) {
					SC_REPORT_ERROR("CPU base", "Read memory");
				}

				if (trans->is_dmi_allowed()) {
					dmi_ptr_valid = instr_bus->get_direct_mem_ptr(*trans,
							dmi_data);
					if (dmi_ptr_valid) {
						std::cout << "Get DMI_PTR " << std::endl;
						dmi_ptr = dmi_data.get_dmi_ptr();
					}
				}
			}

			perf->codeMemoryRead();

			log->SC_log(Log::INFO) << "PC: 0x" << std::hex
					<< register_bank->getPC() << ". ";

			inst->setInstr(INSTR);

			/* check what type of instruction is and execute it */
			switch (inst->check_extension()) {
			[[likely]] case BASE_EXTENSION:
				PC_not_affected = exec->process_instruction(inst);
				incPCby2 = false;
				break;
			case C_EXTENSION:
				PC_not_affected = c_inst->process_instruction(inst);
				incPCby2 = true;
				break;
			case M_EXTENSION:
				PC_not_affected = m_inst->process_instruction(inst);
				incPCby2 = false;
				break;
			case A_EXTENSION:
				PC_not_affected = a_inst->process_instruction(inst);
				incPCby2 = false;
				break;
			[[unlikely]] default:
				std::cout << "Extension not implemented yet" << std::endl;
				inst->dump();
				exec->NOP();
			}

			perf->instructionsInc();

			if (PC_not_affected == true) {
				register_bank->incPC(incPCby2);
			}

			/* Process IRQ (if any) */
			cpu_process_IRQ();

			/* Fixed instruction time to 10 ns (i.e. 100 MHz) */
			//#define USE_QK
#ifdef USE_QK
			// Model time used for additional processing
			m_qk->inc(default_time);
			if (m_qk->need_sync()) {
				m_qk->sync();
			}
#else
			sc_core::wait(default_time);
#endif
		} // while(1)
	} // CPU_thread

	MemoryInterface *mem_intf;

private:

	/**
	 * @brief callback for IRQ simple socket
	 * @param trans transaction to perform (empty)
	 * @param delay time to annotate
	 *
	 * it triggers an IRQ when called
	 */
	void call_interrupt(tlm::tlm_generic_payload &trans,
			sc_core::sc_time &delay) {
		interrupt = true;
		/* Socket caller send a cause (its id) */
		memcpy(&int_cause, trans.get_data_ptr(), sizeof(uint32_t));
		delay = sc_core::SC_ZERO_TIME;
	}

	/**
	 * DMI pointer is not longer valid
	 * @param start memory address region start
	 * @param end memory address region end
	 */
	void invalidate_direct_mem_ptr(sc_dt::uint64 start, sc_dt::uint64 end) {
		(void) start;
		(void) end;
		dmi_ptr_valid = false;
	}

	Registers<T> *register_bank;
	Performance *perf;
	Log *log;
	Instruction *inst;
	C_extension<T> *c_inst;
	M_extension<T> *m_inst;
	A_extension<T> *a_inst;
	BASE_ISA<T> *exec;

	tlm_utils::tlm_quantumkeeper *m_qk;

	bool interrupt;
	uint32_t int_cause;
	bool irq_already_down;
	sc_core::sc_time default_time;
	bool dmi_ptr_valid;
};

#endif
