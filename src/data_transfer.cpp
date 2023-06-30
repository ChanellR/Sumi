#include "arm.hpp"

namespace ARM {

void DataTransferInstruction(Core& cpu, Instruction inst){
  //Memory Transfers
    switch(inst.mnemonic){
        case LDRH: case STRH: case LDRSB: case LDRSH: case LDR: case STR: {
            
            MemOp transfer_instruction;

            if(GetOperatingState(cpu)){
                u16 Rd;
                // printf("format: %d\n", inst.format);
                switch(inst.format){
                    case 6: {
                        //bit 1 of PC is forced to zero
                        transfer_instruction = {(cpu.rf[PC] & ~0x2) + (inst.attributes.t_Word8 << 2), ldw, 0};
                        Rd = inst.attributes.t_f6_Rd;
                        // printf("addr: %X\n", transfer_instruction.addr);
                        break;
                    }
                    case 7: {
                        transfer_instruction.addr = cpu.rf[inst.attributes.t_Ro] + cpu.rf[inst.attributes.t_Rb];
                        switch(inst.attributes.t_L << 1 | inst.attributes.t_f7_B) {
                            case 0B00: {
                                transfer_instruction.data = cpu.rf[inst.attributes.t_Rd];
                                transfer_instruction.operation = strw;
                                break;
                            }
                            case 0B01: {
                                transfer_instruction.data = cpu.rf[inst.attributes.t_Rd];
                                transfer_instruction.operation = strb;
                                break;
                            }
                            case 0B10: {
                                transfer_instruction.operation = ldw;
                                break;
                            }
                            case 0B11: {
                                transfer_instruction.operation = ldb;
                                break;
                            }
                            Rd = inst.attributes.t_Rd;
                            break;
                        }
                        break;
                    }
                    case 8: {
                        transfer_instruction.addr = cpu.rf[inst.attributes.t_Ro] + cpu.rf[inst.attributes.t_Rb];
                        switch((uint8_t(inst.attributes.t_f8_S) << 1) | inst.attributes.t_H) {
                            case 0B00: {
                                transfer_instruction.data = cpu.rf[inst.attributes.t_Rd];
                                transfer_instruction.operation = strh;
                                break;
                            }
                            case 0B01: {
                                transfer_instruction.operation = ldh;
                                break;
                            }
                            case 0B10: {
                                transfer_instruction.operation = ldsb;
                                break;
                            }
                            case 0B11: {
                                transfer_instruction.operation = ldsh;
                                break;
                            }
                        }
                        Rd = inst.attributes.t_Rd;
                        // printf("addr: %X op: %i\n", transfer_instruction.addr, transfer_instruction.operation);
                        break;
                    }
                    case 9: {
                        transfer_instruction.addr = (inst.attributes.t_Offset5 << 2) + cpu.rf[inst.attributes.t_Rb];
                        switch(inst.attributes.t_f910_B << 1 | inst.attributes.t_L) {
                            case 0B00: {
                                transfer_instruction.data = cpu.rf[inst.attributes.t_Rd];
                                transfer_instruction.operation = strw;
                                break;
                            }
                            case 0B01: {
                                transfer_instruction.operation = ldw;
                                break;
                            }
                            case 0B10: {
                                transfer_instruction.data = cpu.rf[inst.attributes.t_Rd];
                                transfer_instruction.operation = strb;
                                break;
                            }
                            case 0B11: {
                                transfer_instruction.operation = ldb;
                                break;
                            }
                        }
                        Rd = inst.attributes.t_Rd;
                        break;
                    }
                    case 10: {
                        Rd = inst.attributes.t_Rd;
                        transfer_instruction.addr = cpu.rf[inst.attributes.t_Rb] + (inst.attributes.t_Offset5 << 1);
                        transfer_instruction.operation = (inst.attributes.t_L) ? ldh : strh;
                        transfer_instruction.data = cpu.rf[inst.attributes.t_Rd];
                        // printf("addr: %X, op: %i, data: %X\n", transfer_instruction.addr, transfer_instruction.operation, transfer_instruction.data);
                        break;
                    }
                    case 11: {
                        Rd = inst.attributes.t_f11_Rd;
                        transfer_instruction.data = cpu.rf[inst.attributes.t_f11_Rd];
                        transfer_instruction.addr = cpu.rf[SP] + (inst.attributes.t_Word8 << 2);
                        transfer_instruction.operation = (inst.attributes.t_L) ? ldw : strw;
                        break;
                    }
                }   

                u32 data = cpu.MMU(transfer_instruction);
                if(inst.mnemonic != STR && inst.mnemonic != STRH) cpu.rf[Rd] = data;
                return;

            } 
            
            u32 addr = GetMemAddress(cpu, inst);
            if(inst.mnemonic == STR) addr &= ~(0x3);
            if(inst.mnemonic == STRH) addr &= ~(0x1); 

            OpType op = (OpType)((inst.attributes.ldst_L << 1) | inst.attributes.ldst_B);
            switch(inst.mnemonic){
                case LDRH: op = ldh; break; 
                case STRH: op = strh; break;
                case LDRSB: op = ldsb; break;
                case LDRSH: op = ldsh; break;
            }
        
            transfer_instruction.operation = op;
            transfer_instruction.data = cpu.rf[inst.attributes.ldst_Rd]; //for store operation
            if(inst.attributes.ldst_Rd == PC) transfer_instruction.data += 4;

            if(inst.attributes.ldst_P){
                transfer_instruction.addr = addr;
            } else {
                transfer_instruction.addr = cpu.rf[inst.attributes.ldst_Rn];
                cpu.rf[inst.attributes.ldst_Rn] = addr;
            }

            u32 data;


            if(((addr & 0x3) && inst.mnemonic == LDR) || ((addr & 0x1) &&  (inst.mnemonic == LDRH || inst.mnemonic == LDRSH))){ //misaligned
                u16 shifts = 0;
                if(inst.mnemonic == LDR) {
                    shifts = transfer_instruction.addr & 0x3;
                    transfer_instruction.addr &= ~(0x3);
                } else if (inst.mnemonic == LDRH) {
                    shifts = transfer_instruction.addr & 0x1;
                    transfer_instruction.addr &= ~(0x1);
                } else {
                    transfer_instruction.operation = ldsb;
                }
                data = ShiftOperation(cpu, rr, cpu.MMU(transfer_instruction), shifts * 8).first;
            } else {
                data = cpu.MMU(transfer_instruction);
            }

            if(inst.attributes.ldst_W){
                cpu.rf[inst.attributes.ldst_Rn] = addr;
            }
            
            if(inst.mnemonic != STR && inst.mnemonic != STRH){
                cpu.rf[inst.attributes.ldst_Rd] = data;
                if(inst.attributes.ldst_Rd == PC) Flush(cpu);
            }
            return;
        }
        case LDM: case STM: {
            // What registers
            // Whenever R15 is stored to memory the stored value is the address of the STM
            // instruction plus 12.
            if(GetOperatingState(cpu)){
                // printf("format: %d\n", inst.format);
                switch(inst.format){
                    case 14: { 
                        MemOp transfer {cpu.rf[SP], (inst.attributes.t_L) ? ldw : strw, 0};
                        if(!inst.attributes.t_L){ //PUSH
                            if(inst.attributes.t_f14_R) {
                                transfer.addr -= 0x4;
                                transfer.data = cpu.rf[LR];
                                cpu.MMU(transfer);
                            }
                            for(int reg = 7; reg >= 0 ; reg--){
                                if(TESTBIT(inst.attributes.t_Rlist, reg)){
                                    //transfer from the where Op1 is pointing for this register
                                    transfer.addr -= 0x4;
                                    transfer.data = cpu.rf[reg];
                                    cpu.MMU(transfer);
                                }
                            }

                        } else { //POP
                            for(int reg = 0; reg < 8 ; reg++){
                                if(TESTBIT(inst.attributes.t_Rlist, reg)){
                                    //transfer from the where Op1 is pointing for this register
                                    cpu.rf[reg] = cpu.MMU(transfer);
                                    transfer.addr += 0x4;
                                }
                            }
                            if(inst.attributes.t_f14_R) {
                                cpu.rf[PC] = cpu.MMU(transfer);
                                Flush(cpu);
                                transfer.addr += 0x4;
                            }
                        }
                        cpu.rf[SP] = transfer.addr;
                        return;
                    }
                    case 15: {
                        MemOp transfer {cpu.rf[inst.attributes.t_f15_Rb], (inst.attributes.t_L) ? ldw : strw, 0};
                        if(inst.attributes.t_L){ //LDM 
                            for(int reg = 7; reg >= 0 ; reg--){
                                if(TESTBIT(inst.attributes.t_Rlist, reg)){
                                    //transfer from the where Op1 is pointing for this register
                                    cpu.rf[reg] = cpu.MMU(transfer);
                                    transfer.addr += 0x4;
                                }
                            }
                        } else { //STM
                            for(int reg = 0; reg < 8 ; reg++){
                                if(TESTBIT(inst.attributes.t_Rlist, reg)){
                                    //transfer from the where Op1 is pointing for this register
                                    transfer.data = cpu.rf[reg]; 
                                    cpu.MMU(transfer);
                                    transfer.addr += 0x4;
                                }
                            }
                        }
                        cpu.rf[inst.attributes.t_f15_Rb] = transfer.addr;
                        return;
                    }
                }
            }

            u32 base = cpu.rf[inst.attributes.ldstM_Rn];
            MemOp transfer {base, (inst.attributes.ldstM_L) ? ldw : strw, 0};
            if(inst.attributes.ldstM_register_list == 0x0){
                transfer.data = cpu.rf[PC] + 4;
                if(inst.mnemonic == LDM){
                    if(inst.attributes.ldstM_P){
                        cpu.rf[inst.attributes.ldstM_Rn] += (inst.attributes.ldstM_U) ? 0x40 : -0x40;
                        transfer.addr = cpu.rf[inst.attributes.ldstM_Rn];
                        cpu.rf[PC] = cpu.MMU(transfer);
                        Flush(cpu);
                    } else {
                        cpu.rf[PC] = cpu.MMU(transfer);
                        cpu.rf[inst.attributes.ldstM_Rn] += (inst.attributes.ldstM_U) ? 0x40 : -0x40;
                        Flush(cpu);
                    }
                } else {
                    if(inst.attributes.ldstM_P){ //pre
                        cpu.rf[inst.attributes.ldstM_Rn] += (inst.attributes.ldstM_U) ? 0x40 : -0x40;
                        transfer.addr += (inst.attributes.ldstM_U) ? 0x4 : -0x40;
                        cpu.MMU(transfer);
                    } else {
                        transfer.addr += (inst.attributes.ldstM_U) ? 0x0 : -0x3C;
                        cpu.MMU(transfer);
                        cpu.rf[inst.attributes.ldstM_Rn] += (inst.attributes.ldstM_U) ? 0x40 : -0x40;
                    }
                }
                return;
            }

            bool copy_context = false;
            bool ret_context = false;
            u32 current_context = cpu.rf[CPSR];

            if((inst.attributes.ldstM_register_list & (0x1 << 15)) && inst.attributes.ldstM_S){ //R15 in transfer list
                if(inst.mnemonic == LDM){
                    copy_context = true;
                } else {
                    ret_context = true;
                    cpu.rf[CPSR] = (cpu.rf[CPSR] & ~(0x1F)) | (0x10); //go user mode;
                }
            } else if (!(inst.attributes.ldstM_register_list & (0x1 << 15)) && inst.attributes.ldstM_S) {
                ret_context = true;
                cpu.rf[CPSR] = (cpu.rf[CPSR] & ~(0x1F)) | (0x10); //go user mode;
            }
           
            if(inst.attributes.ldstM_U){
                u32 base_addr = 0; //for writeback edgecases
                for(int reg = 0; reg < 16; reg++){
                    if(TESTBIT(inst.attributes.ldstM_register_list, reg)){
                        //transfer from the where Op1 is pointing for this register
                        transfer.data = cpu.rf[reg];
                        if(reg == 15) transfer.data += 4; //12 ahead
                        if(inst.attributes.ldstM_P){
                            transfer.addr += 0x4;
                            u32 out = cpu.MMU(transfer);
                            if(reg == inst.attributes.ldstM_Rn) base_addr = transfer.addr;
                            if(inst.attributes.ldstM_L) cpu.rf[reg] = out;
                        } else {
                            u32 out = cpu.MMU(transfer);
                            if(reg == inst.attributes.ldstM_Rn) base_addr = transfer.addr;
                            if(inst.attributes.ldstM_L) cpu.rf[reg] = out;
                            transfer.addr += 0x4;
                        }
                        if(inst.attributes.ldstM_L && reg == 15) Flush(cpu); //flush if changing PC
                    }
                }

                /*
                   Writeback with Rb included in Rlist: Store OLD base if Rb is FIRST entry in Rlist,
                   otherwise store NEW base (STM/ARMv4), always store OLD base (STM/ARMv5),
                   no writeback (LDM/ARMv4), writeback if Rb is "the ONLY register,
                   or NOT the LAST register" in Rlist (LDM/ARMv5).
                */

                if(inst.attributes.ldstM_W) {
                    //don't overwrite the load
                    bool Rb_first_in_list = ((1 << inst.attributes.ldstM_Rn) & inst.attributes.ldstM_register_list)
                    && !(((1 << inst.attributes.ldstM_Rn) - 1) & inst.attributes.ldstM_register_list);

                    if(!(inst.mnemonic == LDM && ((1 << inst.attributes.ldstM_Rn) & inst.attributes.ldstM_register_list))){
                        cpu.rf[inst.attributes.ldstM_Rn] = transfer.addr;
                    } 
                    if (inst.mnemonic == STM) {
                        // cpu.rf[inst.attributes.ldstM_Rn] = transfer.addr;
                        if(!Rb_first_in_list) {
                            MemOp storing_new_base {base_addr, strw, cpu.rf[inst.attributes.ldst_Rn]};
                            cpu.MMU(storing_new_base);
                        }
                    }

                }

            } else {
                u32 base_addr = 0; //for writeback edgecases
                for(int reg = 15; reg >= 0; reg--){
                    if(TESTBIT(inst.attributes.ldstM_register_list, reg)){
                        //transfer from the where Rn is pointing for this register
                        transfer.data = cpu.rf[reg];
                        if(reg == 15) transfer.data += 4; //12 ahead
                        if(inst.attributes.ldstM_P){
                            transfer.addr -= 0x4;
                            u32 out = cpu.MMU(transfer);
                            if(reg == inst.attributes.ldstM_Rn) base_addr = transfer.addr;
                            if(inst.attributes.ldstM_L) cpu.rf[reg] = out;
                        } else {
                            u32 out = cpu.MMU(transfer);
                            if(reg == inst.attributes.ldstM_Rn) base_addr = transfer.addr;
                            if(inst.attributes.ldstM_L) cpu.rf[reg] = out;
                            transfer.addr -= 0x4;
                        }
                        if(inst.attributes.ldstM_L && reg == 15) Flush(cpu); //flush if changing PC
                    }
                }
                
                if(inst.attributes.ldstM_W) {
                    //don't overwrite the load
                    bool Rb_first_in_list = ((1 << inst.attributes.ldstM_Rn) & inst.attributes.ldstM_register_list)
                    && !(((1 << inst.attributes.ldstM_Rn) - 1) & inst.attributes.ldstM_register_list);

                    if(!(inst.mnemonic == LDM && ((1 << inst.attributes.ldstM_Rn) & inst.attributes.ldstM_register_list))){
                        cpu.rf[inst.attributes.ldstM_Rn] = transfer.addr;
                    } 
                    if (inst.mnemonic == STM) {
                        // cpu.rf[inst.attributes.ldstM_Rn] = transfer.addr;
                        if(!Rb_first_in_list) {
                            MemOp storing_new_base {base_addr, strw, cpu.rf[inst.attributes.ldst_Rn]};
                            cpu.MMU(storing_new_base);
                        }
                    }

                }

            }
            
            if(ret_context) cpu.rf[CPSR] = current_context;
            if(copy_context) cpu.rf[CPSR] = cpu.rf[SPSR];

            return;
        }
        case SWP:{
            u32 addr = cpu.rf[inst.attributes.ldst_Rn];
            u16 shifts = 0;
            if(addr & 0x3 && !inst.attributes.ldst_B){
                shifts = (addr & 0x3) * 8;
                addr &= ~(0x3);
            }
            MemOp load {addr, (inst.attributes.ldst_B) ? ldb : ldw, 0};
            MemOp store {addr, (inst.attributes.ldst_B) ? strb : strw, cpu.rf[inst.attributes.ldst_Rm]};
            if(!inst.attributes.ldst_B && shifts != 0) {
                u32 data = cpu.MMU(load);
                cpu.rf[inst.attributes.ldst_Rd] = ShiftOperation(cpu, rr, data, shifts).first;
                // printf("data: %X reg: %X shifts: %i\n", data, cpu.rf[inst.attributes.ldst_Rd], shifts);
            } else {
                cpu.rf[inst.attributes.ldst_Rd] = cpu.MMU(load);
            }
            cpu.MMU(store);
            return;
        }
    }
}

}; //Namespace ARM