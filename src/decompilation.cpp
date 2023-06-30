#include "arm.hpp"
#include "bit.hpp"
#include <tuple>

namespace ARM {

ARM::DisplayType GetDisplayType(InstructionMnemonic mnemonic) {

    switch(mnemonic){
        case MOV: case MVN: case MRS: case MSR: case MSRf: {
            return dp_1;
        }
        case TST: case TEQ: case CMP: case CMN:{
            return dp_2;
        }
        case AND: case EOR: case SUB: case RSB:
        case ADD: case ADC: case SBC: case RSC:
        case ORR: case BIC: {
            return dp_3;
        }
        case B: return b_1;
        case BX: return b_2;
        case LDR: case STR: case LDRH: case STRH: case LDRSB: case LDRSH: {
            return ldst;
        }
        case STM: case LDM: {
            return ldstM;
            break;
        }
        case SWP:{
            return swp;
        }
        case MUL: case MLA: case MULL: case MLAL: {
            return mul;
        }
        default:
            return NOOP;
    }
}

void Info(Core& cpu, Instruction inst){
    char Disassembly[20];
    DisplayType type = GetDisplayType(inst.mnemonic);
    Disassemble(cpu, Disassembly, inst.attributes.word, inst.address, false);
    printf("instruction: 0x%X %s\n",inst.attributes.word, Disassembly);
    printf("mnemonic: %d display type: %d\n", inst.mnemonic, type);
}

void Disassemble(Core& cpu, char* buffer, u32 instruction, u32 instruction_addr, bool Thumb){
    //for translating into readable syntax

    InstructionMnemonic mnem;
    u16 format;
    std::tie(mnem, format) = (Thumb) ? Decode_T(instruction) : Decode(instruction);
    Instruction inst {mnem, instruction, instruction_addr, format};
    DisplayType type = GetDisplayType(inst.mnemonic);


#define HEADERBEGIN 0x08000000
#define HEADEREND 0x080000C0

    if(inst.address > HEADERBEGIN && inst.address < HEADEREND) {
        sprintf(buffer, "dd");
        return;
    }

#undef HEADERBEGIN 
#undef HEADEREND 

    const char* instruction_stems[] {
        "AND","EOR","SUB","RSB","ADD","ADC","SBC","RSC",
        "TST","TEQ","CMP","CMN","ORR","MOV","BIC","MVN",
        "B","UNDEF","UNIMP", "MRS", "MSR", "BX", "LDR",
        "STR", "SWI", "LDRH", "STRH", "LDRSB", "LDRSH", 
        "LDM", "STM", "MSRf", "SWP", "MUL", "MLA", "MULL", "MLAL"
    };

    const char* conditions_extensions[] {
        "EQ", //equal
        "NE", //not equal
        "CS", //unsigned higher or same
        "CC", //unsigned lower
        "MI", //negative
        "PL", //positive or zero
        "VS", //overflow
        "VC", //no overflow
        "HI", //unsigned higher
        "LS", //unsigned lower or same
        "GE", //greater or equal
        "LT", //less than
        "GT", //greater than
        "LE", //less than or equal
        "", //always
        "error_cond"
    };

    const char* shift_type[]{"lsl", "lsr", "asr", "rr"};

    //THUMB
    if(Thumb){
        switch(inst.format){
            case 1: {
                sprintf(buffer, "%s r%u, r%u, %Xh",
                shift_type[inst.attributes.t_f1_Opcode],
                inst.attributes.t_Rd, inst.attributes.t_Rs,
                inst.attributes.t_Offset5);
                return;
            }
            case 2: {
                sprintf(buffer, "%sS r%u, r%u, %s%X%s",
                instruction_stems[inst.mnemonic],
                inst.attributes.t_Rd, inst.attributes.t_Rs,
                (inst.attributes.t_I) ? "" : "r",
                inst.attributes.t_Rn_Offset3, 
                (inst.attributes.t_I) ? "h" : "");
                return;
            }
            case 3: {
                sprintf(buffer, "%sS r%u, %Xh",
                instruction_stems[inst.mnemonic],
                inst.attributes.t_f3_Rd, inst.attributes.t_Offset8);
                return;
            }
            case 4: {
                switch(inst.mnemonic){
                    case TST: case RSB: case CMP: case CMN: case MVN: {
                        sprintf(buffer, "%s r%u, r%u",
                        instruction_stems[inst.mnemonic],
                        inst.attributes.t_Rd,
                        inst.attributes.t_Rs);
                        break;
                    }
                    default: {
                        sprintf(buffer, "%s r%u, r%u, r%u",
                        instruction_stems[inst.mnemonic],
                        inst.attributes.t_Rd,
                        inst.attributes.t_Rd,
                        inst.attributes.t_Rs);

                    }
                }
                return;
            }
            case 5: {
                if(inst.attributes.t_f5_Opcode ==  0B11) {
                    sprintf(buffer, "BX r%u",
                        (inst.attributes.t_H2) ? 0x8 | inst.attributes.t_RHs : inst.attributes.t_RHs);
                    return;
                }
                sprintf(buffer, "%s r%u r%u", 
                instruction_stems[inst.mnemonic],
                (inst.attributes.t_H1) ? 0x8 | inst.attributes.t_RHd : inst.attributes.t_RHd,
                (inst.attributes.t_H2) ? 0x8 | inst.attributes.t_RHs : inst.attributes.t_RHs);
                return;
            }
            case 6: {
                sprintf(buffer, "LDR r%u, [PC, %Xh]",
                inst.attributes.t_f6_Rd, inst.attributes.t_Word8 << 2);
                return;
            }
            case 7: {
                sprintf(buffer, "%s r%u, [r%u, r%u]",
                instruction_stems[inst.mnemonic],
                inst.attributes.t_Rd,
                inst.attributes.t_Rb,
                inst.attributes.t_Rd);
                return;
            }
            case 15: {
                sprintf(buffer, "%s r%u!, {0x%0X}",
                instruction_stems[inst.mnemonic], inst.attributes.t_f15_Rb,
                inst.attributes.t_Rlist);
                return;
            }
            case 16: {
                sprintf(buffer, "B%s [PC, %X]",
                conditions_extensions[inst.attributes.t_Cond],
                inst.attributes.t_Soffset8 << 1);
                return;
            }
            case 17: {
                sprintf(buffer, "SWI %X", inst.attributes.t_Word8);
                return;
            }
            case 18: {
                sprintf(buffer, "B %X", inst.attributes.t_Offset11 << 1);
                return;
            }
            case 19: {
                sprintf(buffer, "LongB %s %X", (inst.attributes.t_L) ? "low" : "high",
                 inst.attributes.t_Offset);
                return;
            }
        }
        sprintf(buffer, "%s%s", instruction_stems[inst.mnemonic],
        (inst.mnemonic != B) ? "S" : "");
        return;
    }

    //ARM
    switch(type){
        case dp_1:{
            u16 Rd = inst.attributes.dp_Rd;
            u16 Rm = inst.attributes.dp_Rm;
            if(inst.mnemonic == MSR) Rd = CPSR;
            if(inst.mnemonic == MRS) Rm = CPSR;

            if(!inst.attributes.dp_I){
                if (inst.attributes.dp_reg_shift){

                    sprintf(buffer, "%s%s%s r%u, r%u, %s r%u",
                        instruction_stems[inst.mnemonic],
                        conditions_extensions[inst.attributes.Cond],
                        (inst.attributes.dp_S) ? "S" : "",
                        Rd, 
                        Rm,
                        shift_type[inst.attributes.dp_shift_type],
                        inst.attributes.dp_Rs,
                        GetOperand(cpu, inst)
                        );
                } else {

                    sprintf(buffer, "%s%s%s r%u, r%u, %s %Xh=%Xh",
                        instruction_stems[inst.mnemonic],
                        conditions_extensions[inst.attributes.Cond],
                        (inst.attributes.dp_S) ? "S" : "",
                        Rd, 
                        Rm,
                        shift_type[inst.attributes.dp_shift_type],
                        inst.attributes.dp_shift_amount,
                        GetOperand(cpu, inst)
                        );
                }
            } else {
                sprintf(buffer, "%s%s%s r%u, %Xh",
                    instruction_stems[inst.mnemonic],
                    conditions_extensions[inst.attributes.Cond],
                    (inst.attributes.dp_S) ? "S" : "",
                    Rd, GetOperand(cpu, inst));
            }
            break;
        }
        case dp_2:{
            if(!inst.attributes.dp_I && inst.attributes.dp_Shift == 0){
                sprintf(buffer, "%s%s r%u, r%u",
                instruction_stems[inst.mnemonic],
                conditions_extensions[inst.attributes.Cond],
                inst.attributes.dp_Rn, inst.attributes.dp_Rm);
            } else {
                sprintf(buffer, "%s%s r%u, %Xh",
                instruction_stems[inst.mnemonic],
                conditions_extensions[inst.attributes.Cond],
                inst.attributes.dp_Rn, GetOperand(cpu, inst));
            }
            break;
        }
        case dp_3:{
            if(!inst.attributes.dp_I ){
                sprintf(buffer, "%s%s%s r%u, r%u, r%u, %s %Xh",
                instruction_stems[inst.mnemonic],
                conditions_extensions[inst.attributes.Cond],
                (inst.attributes.dp_S) ? "S" : "",
                inst.attributes.dp_Rd, inst.attributes.dp_Rn, inst.attributes.dp_Rm,
                shift_type[inst.attributes.dp_shift_type],
                inst.attributes.dp_shift_amount
                );
            } else {
                sprintf(buffer, "%s%s%s r%u, r%u, %Xh",
                instruction_stems[inst.mnemonic],
                conditions_extensions[inst.attributes.Cond],
                (inst.attributes.dp_S) ? "S" : "",
                inst.attributes.dp_Rd, inst.attributes.dp_Rn, GetOperand(cpu, inst));
            }
            break;
        }
        case b_1: {
            sprintf(buffer, "B%s%s pc, %0Xh =%0Xh %s", //off by one sometimes, PIPELINE!
                    (inst.attributes.branch_link) ? "L" : "",
                    conditions_extensions[inst.attributes.Cond],
                    inst.attributes.branch_offset,
                    (u32)(inst.address + (inst.attributes.branch_offset << 2) + 8),
                    (Condition(cpu, inst.attributes.Cond)) ? "true;" : "false;"
                ); 
            break;
        }
        case b_2:{
            if(cpu.rf[inst.attributes.branch_Rn] & 0x1){
                sprintf(buffer, "BX%s [r%u] =%Xh THUMB",
                conditions_extensions[inst.attributes.Cond],
                 inst.attributes.branch_Rn, cpu.rf[inst.attributes.branch_Rn]);
            } else {
                sprintf(buffer, "BX%s [r%u] =%Xh ARM",
                conditions_extensions[inst.attributes.Cond],
                 inst.attributes.branch_Rn, cpu.rf[inst.attributes.branch_Rn]);
            }
            break;
        }
        case ldst: {
            buffer += sprintf(buffer, "%s%s%s r%u, [r%u, %s%Xh]", 
                    instruction_stems[inst.mnemonic],
                    conditions_extensions[inst.attributes.Cond],
                    (inst.attributes.ldst_B && (inst.mnemonic == STR || inst.mnemonic == LDR)) ? "B" : "",
                    inst.attributes.ldst_Rd,
                    inst.attributes.ldst_Rn,
                    (inst.attributes.ldst_U) ? "" : "-",
                    GetMemAddress(cpu, inst, true)
                );
            if(cpu.rf[PC] <= inst.address + 2) {
                sprintf(buffer, "=%08Xh",
                 (inst.attributes.ldst_P) ? GetMemAddress(cpu, inst) : cpu.rf[inst.attributes.ldst_Rn]);
            }
            break;
        }
        case ldstM: {
            sprintf(buffer, "%s%s r%u%s, {0x%X}",
                instruction_stems[inst.mnemonic],
                conditions_extensions[inst.attributes.Cond],
                inst.attributes.ldstM_Rn,
                (inst.attributes.ldstM_W) ? "!" : "",
                inst.attributes.ldstM_register_list
            );

            return;
        }
        case swp:{
            sprintf(buffer, "%s%s%s r%u, r%u, [r%u]",
                instruction_stems[inst.mnemonic],
                conditions_extensions[inst.attributes.Cond],
                (inst.attributes.ldst_B) ? "B" : "",
                inst.attributes.ldst_Rd, 
                inst.attributes.ldst_Rm,
                inst.attributes.ldst_Rn
            );
            return;
        }
        case mul: {
            if (inst.mnemonic == MLAL || inst.mnemonic == MULL) {
                sprintf(buffer, "%s%s%s%s r%u, r%u, r%u, r%u",
                        (inst.attributes.mull_U) ? "S" : "U",
                        instruction_stems[inst.mnemonic],
                        conditions_extensions[inst.attributes.Cond],
                        (inst.attributes.mull_S) ? "S" : "",
                        inst.attributes.mull_RdLo,
                        inst.attributes.mull_RdHi,
                        inst.attributes.mull_Rm,
                        inst.attributes.mull_Rs
                    );
                break;
            } 
            buffer += sprintf(buffer, "%s%s%s r%u, r%u, r%u",
                        instruction_stems[inst.mnemonic],
                        conditions_extensions[inst.attributes.Cond],
                        (inst.attributes.mul_S) ? "S" : "",
                        inst.attributes.mul_Rd,
                        inst.attributes.mul_Rm,
                        inst.attributes.mul_Rs);
            if(inst.mnemonic == MLA) sprintf(buffer, " ,r%u", inst.attributes.mul_Rn);
            break;
        }
        case NOOP: {
            sprintf(buffer, "NOOP"); 
            break;
        }
    }
}

}; //Namespace ARM