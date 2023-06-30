#include "arm.hpp"
#include <tuple>

namespace ARM {

void DataProcessingInstruction(Core& cpu, Instruction inst){
    
    u32 result;
    u32 Op1, Op2;
    u8 C, Rd;
    bool force_S = false;

  
    if(GetOperatingState(cpu)) {
        //go through a different process of getting Operands
        C = CARRY_VALUE;
        force_S = true;
        switch(inst.format){
            case 1: {
                Op1 = 0;
                Rd = inst.attributes.t_Rd;
                // printf("before\n");
                std::tie(Op2, C) = ShiftOperation(cpu, ShiftOp(inst.attributes.t_f1_Opcode), 
                                                    cpu.rf[inst.attributes.t_Rs], 
                                                    inst.attributes.t_Offset5);
                // printf("Op1: %X Op2: %X, Rd: %u, after\n", Op1, Op2, Rd);
                break;
            }
            case 4: {
                Rd = inst.attributes.t_Rd;
                Op1 = cpu.rf[inst.attributes.t_Rd];
                switch(inst.mnemonic){
                    case MOV: {
                        ShiftOp op;
                        switch(inst.attributes.t_f4_Opcode){
                            case 0B0010: op = lsl; break;
                            case 0B0011: op = lsr; break;
                            case 0B0100: op = asr; break;
                            case 0B0111: op = rr; break;
                        }
                        //register specified shifting
                        std::tie(Op2, C) = ShiftOperation(cpu, op, 
                                                        cpu.rf[inst.attributes.t_Rd], 
                                                        cpu.rf[inst.attributes.t_Rs],
                                                        true);
                        // printf("Op2: %X\n", Op2);
                        break;
                    }
                    case RSB: {
                        Op1 = cpu.rf[inst.attributes.t_Rs];
                        Op2 = 0;
                        break;
                    }
                    default: {
                        Op2 = cpu.rf[inst.attributes.t_Rs];
                        break;
                    }
                }
                break; 
            }
            case 2: {
                Op1 = cpu.rf[inst.attributes.t_Rs];
                Rd = inst.attributes.t_Rd;
                if(inst.attributes.t_I) {
                    Op2 = inst.attributes.t_Rn_Offset3;
                } else {
                    Op2 = cpu.rf[inst.attributes.t_Rn_Offset3];
                }
                break;
            }
            case 3: {
                Rd = inst.attributes.t_f3_Rd;
                Op1 = cpu.rf[inst.attributes.t_f3_Rd];
                Op2 = inst.attributes.t_Offset8;
                break;
            }
            case 5: {
                Rd = inst.attributes.t_RHd;
                Rd |= (inst.attributes.t_H1) ? 0x8 : 0;
                uint16_t Rs = inst.attributes.t_RHs;
                Rs |= (inst.attributes.t_H2) ? 0x8 : 0;
                Op1 = cpu.rf[Rd];
                Op2 = cpu.rf[Rs];
                if(inst.attributes.t_f5_Opcode != 1) force_S = false; //only on CMP
                break;
            }
            case 12: {
                force_S = false; //just ADD
                Rd = inst.attributes.t_f12_Rd;
                Op1 = (inst.attributes.t_SP) ? cpu.rf[SP] : (cpu.rf[PC] & ~2); //aligns to 32 bit address
                Op2 = (inst.attributes.t_Word8 << 2);
                // printf("Op1: %X Op2: %X\n", Op1, Op2);
                break;
            }
            case 13: {
                force_S = false;
                Rd = SP;
                Op1 = cpu.rf[SP];
                Op2 = inst.attributes.t_SWord7 << 2;
                Op1 += (inst.attributes.t_f13_S) ? -1 * i32(Op2) : Op2; //bandaid solution
                Op2 = 0;
                break;
            }
        }

    } else {
        Op1 = cpu.rf[inst.attributes.dp_Rn];
        //extra time spent in shifting regisiter, creating diff in PC
        if(inst.attributes.dp_Rn == PC && !inst.attributes.dp_I && inst.attributes.dp_reg_shift) Op1 += 4; 
        Rd = inst.attributes.dp_Rd;
        std::tie(Op2, C) = GetOperand(cpu, inst);
        // printf("Op1:%X Op2:%X C:%X\n", Op1, Op2, C);
    }

    InstructionMnemonic mnemonic = inst.mnemonic;
    
    u8 old_carry = C;

    //Data Processing
    switch(mnemonic){
        //Subtraction based alteration
        case CMP: case SUB: case SBC:{
            Op2 = ~Op2;
            C = (mnemonic == SBC) ? C : 1;
            mnemonic = ADC;
            break;
        }
        case RSB: case RSC:{
            Op1 = ~Op1;
            C = (mnemonic == RSC) ? C : 1;
            mnemonic = ADC; 
            break;
        }
    }

    bool logical = false;
    switch (mnemonic)
    {   
        case MRS: cpu.rf[inst.attributes.dp_Rd] = (inst.attributes.msr_P) ? cpu.rf[SPSR] : cpu.rf[CPSR]; return;
        case MSR: { //there is a slight differnce in the docs considering bit16!!
            
            u32 mask = 0;

            if (inst.attributes.status_mask & (1 << 0)) mask |= 0x000000FF;
            if (inst.attributes.status_mask & (1 << 1)) mask |= 0x0000FF00;
            if (inst.attributes.status_mask & (1 << 2)) mask |= 0x00FF0000;
            if (inst.attributes.status_mask & (1 << 3)) mask |= 0xFF000000;
            
            if(PSR{cpu.rf[CPSR]}.operating_mode == 0B10000){ //User mode
                mask &= 0xF0000000;
            }

            if(inst.attributes.msr_P) {
                cpu.rf[SPSR] = (cpu.rf[SPSR] & ~mask) | (Op2 & mask);
            } else {
                cpu.rf[CPSR] = (cpu.rf[CPSR] & ~mask) | (Op2 & mask);
            }
            
            return;
        }
        //Logical
        case MOV: result = Op2; logical = true; break;
        case AND: 
        case TST: result = Op1 & Op2; logical = true; break;
        case BIC: result = Op1 & ~Op2; logical = true; break;
        case EOR: 
        case TEQ: result = Op1 ^ Op2; logical = true; break;
        case MVN: result = ~Op2; logical = true; break;
        case ORR: result = Op1 | Op2; logical = true; break;
        //Arithmetic
        case ADD:
        case CMN: result = Op1 + Op2; break;
        case ADC: {
            result = Op1;
            result += Op2;
            result += (inst.mnemonic == ADC) ? CARRY_VALUE : C; 
            break;
        }
    }

    bool no_write = (inst.mnemonic == TST || inst.mnemonic == TEQ || inst.mnemonic == CMP || inst.mnemonic == CMN);
    
    if(!no_write) {
        cpu.rf[Rd] = result;
    }

    if(Rd == PC) {
        if(GetOperatingState(cpu)) cpu.rf[Rd] &= ~(0x1); //alignment
        if(inst.attributes.dp_S) cpu.rf[CPSR] = cpu.rf[SPSR];
        if(!no_write)Flush(cpu);
    }
    
    if(force_S || (inst.attributes.dp_S && (inst.attributes.dp_Rd != 15))) { //check for THUMB MODE THINGS
        PSR cpsr {cpu.rf[CPSR]};
        if(logical){
            cpsr.C = old_carry;
        } else {
            cpsr.V = (~(Op2 ^ Op1) & (result ^ Op1)) >> 31;
            switch (inst.mnemonic)
            {
                case CMP: case SUB: cpsr.C = (Op1 >= ~Op2); break;
                case SBC: cpsr.C = (Op1 > ~Op2); break;
                case RSB: case RSC: cpsr.C = (Op2 >= ~Op1); break;  
                case ADC: cpsr.C = ((0xFFFFFFFF - Op1 - C) < Op2); break;   
                default: cpsr.C = ((0xFFFFFFFF - Op1) < Op2); break;
            }
        }
        cpsr.Z = (result == 0) ? 1 : 0;
        cpsr.N = (result & (0x1 << 31)) ? 1 : 0;
        cpu.rf[CPSR] = cpsr.word;
    }
}

}; //Namespace ARM