#include "../include/Sumi/arm.hpp"
#include <cstdio>
#include <iostream>
#include <string>
#include <stdint.h>
#include <assert.h>

#include <cstring>

void Arm::reset(){
    //sets all registers to 0
    memset(rf, 0, 4*17);
    //for now
    rf[PC] = 0x08000000; //beginning of ROM
    rf[LR] = 0x08000000; //beginning of ROM
    rf[SP] = 0x03007F00;
    rf[CPSR] = 0x0000001F; //user mode
}

uint32_t Arm::fetch(){
    //just fetching double-words for now
    MemOp fetch_operation = {rf[PC], ldw};
    uint32_t instruction = MMU(fetch_operation);
    rf[PC] += 4;
    return instruction;
}

bool Arm::condition_pass(ConditionField condition){
    switch (condition)
    {
        case AL: 
            return true;
            break;
    }
    return false;
}

void Arm::set_reg(uint16_t reg, uint32_t value){
    rf[reg] = value;
}

Arm::InstructionMnemonic Arm::decode(uint32_t instruction){

    InstructionAttributes inst;
    inst.word = instruction;

    //TODO;
    //MUL, SWP, block transfers, Coprocessor
    
    if(MATCH(instruction & 0xe0000010, 0x60000010)) return UNDEF;
    if(MATCH(instruction & 0x0ffffff0, 0x012FFF10)) return BX;
    if(MATCH(instruction & 0x0F000000, 0x0F000000)) return SWI;
    if (MATCH(instruction & 0x0E000000, 0x0A000000)) return B;


    if(!TESTBIT(instruction, 27) && TESTBIT(instruction, 26)){
        if (TESTBIT(instruction, BIT_loadStore)) {
            return LDR;
        } else return STR;
    }

    //Halfword and Signed Data Transfer && SWP && MUL
    if (inst.ldstx_valid_3 == 0 && inst.ldstx_valid_2 && inst.ldstx_valid_1) {
        if(inst.ldstx_SH == 0){
            //MUL && SWP

        } else if(inst.ldstx_L){
            //load
            if(!inst.ldstx_immediate_type && inst.ldstx_offset_2 == 0) {
                //register offset
                switch(inst.ldstx_SH){
                    case 0B01: return LDRH;
                    case 0B10: return LDRSB; 
                    case 0B11: return LDRSH;
                }
            } else if (inst.ldstx_immediate_type) {
                //immediate offset
                switch(inst.ldstx_SH){
                    case 0B01: return LDRH;
                    case 0B10: return LDRSB; 
                    case 0B11: return LDRSH;
                }
            }

        } else {
            //store
            if(!inst.ldstx_immediate_type && inst.ldstx_offset_2 == 0) {
                //register offset
                return STRH;
            } else if (inst.ldstx_immediate_type) {
                //immediate offset
                return STRH;
            }
        }
    }

    /*
        The data processing operations may be classified as logical or arithmetic. The logical
        operations (AND, EOR, TST, TEQ, ORR, MOV, BIC, MVN) perform the logical action
        on all corresponding bits of the operand or operands to produce the result. If the S bit
        is set (and Rd is not R15, see below) the V flag in the CPSR will be unaffected, the C
        flag will be set to the carry out from the barrel shifter (or preserved when the shift
        operation is LSL #0), the Z flag will be set if and only if the result is all zeros, and the
        N flag will be set to the logical value of bit 31 of the result
    
        The MRS and MSR instructions are formed from a subset of the Data Processing
        operations and are implemented using the TEQ, TST, CMN and CMP instructions
        without the S flag set. The encoding is shown in ➲Figure 4-11: PSR transfer on page
        4-19.
    */

    if(inst.dp_valid == 0){
        //checks for Immediate offset or register with proper_validation_bits
        if(inst.dp_I || ((!inst.dp_reg_padding_bit && inst.dp_reg_shift) || !inst.dp_reg_padding_bit)){
            InstructionMnemonic prelim_type = (InstructionMnemonic)inst.dp_OpCode;
            if(prelim_type == TEQ || prelim_type == TST || prelim_type == CMN || prelim_type == CMP){
                if(!TESTBIT(instruction, BIT_S)){
                    if(MATCH(instruction & 0x0FBF0FFF, 0x010F0000)){
                        return MRS;
                    } else if (MATCH(instruction & 0x0FBFFFF0, 0x0129F000)){
                        return MSR;
                    } 
                }
            } else return prelim_type;
        }
    }

    return UNIMP;
}

void Arm::disassemble(char* buffer, uint32_t instruction, uint32_t instruction_addr){
    //for translating into readable syntax
    InstructionMnemonic mnemonic = decode(instruction);
    DisplayTypes type = get_display_type(mnemonic);

    // printf("mnemonic: %d display type: %d\n", mnemonic, type);
    InstructionAttributes inst;
    inst.word = instruction;

#define HEADERBEGIN 0x08000000
#define HEADEREND 0x080000C0
    if(instruction_addr > HEADERBEGIN && instruction_addr < HEADEREND) {
        sprintf(buffer, "dd");
        return;
    }

    //will add more later
    const char* instruction_stems[30] {
        "AND","EOR","SUB","RSB","ADD","ADC","SBC","RSC",
        "TST","TEQ","CMP","CMN","ORR","MOV","BIC","MVN",
        "B","UNDEF","UNIMP", "MRS", "MSR", "BX", "LDR",
        "STR", "SWI", "LDRH", "STRH", "LDRSB", "LDRSH"
    };

    const char* conditions_extensions[15] {
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
    
    };

    switch(type){
        case dp_1:{
            uint16_t Rd = inst.dp_Rd;
            uint16_t Rm = inst.dp_Rm;
            if(mnemonic == MSR) Rd = CPSR;
            if(mnemonic == MRS) Rm = CPSR;
            
            if(!inst.dp_I && inst.dp_Shift == 0){
                sprintf(buffer, "%s%s r%u, r%u",
                instruction_stems[mnemonic],
                conditions_extensions[inst.Cond],
                Rd, Rm);
            } else {
                sprintf(buffer, "%s%s r%u, %Xh",
                instruction_stems[mnemonic],
                conditions_extensions[inst.Cond],
                Rd, get_operand_2(inst));
            }
            break;
        }
        case dp_2:{
            if(!inst.dp_I && inst.dp_Shift == 0){
                sprintf(buffer, "%s%s r%u, r%u",
                instruction_stems[mnemonic],
                conditions_extensions[inst.Cond],
                inst.dp_Rn, inst.dp_Rm);
            } else {
                sprintf(buffer, "%s%s r%u, %Xh",
                instruction_stems[mnemonic],
                conditions_extensions[inst.Cond],
                inst.dp_Rn, get_operand_2(inst));
            }
            break;
        }
        case dp_3:{
            if(!inst.dp_I && inst.dp_Shift == 0){
                sprintf(buffer, "%s%s r%u, r%u, r%u",
                instruction_stems[mnemonic],
                conditions_extensions[inst.Cond],
                inst.dp_Rd, inst.dp_Rn, inst.dp_Rm);
            } else {
                sprintf(buffer, "%s%s r%u, r%u, %Xh",
                instruction_stems[mnemonic],
                conditions_extensions[inst.Cond],
                inst.dp_Rd, inst.dp_Rn, get_operand_2(inst));
            }
            break;
        }
        case b_1: {
            sprintf(buffer, "B%s%s =%0Xh",
                    (inst.branch_link) ? "L" : "",
                    conditions_extensions[inst.Cond],
                    (uint32_t)(instruction_addr + (inst.branch_offset << 2) + 4)); 
            break;
        }
        case b_2:{
            if(rf[inst.branch_Rn] & 0x1){
                sprintf(buffer, "BX%s [r%u] =%Xh THUMB",
                conditions_extensions[inst.Cond],
                 inst.branch_Rn, rf[inst.branch_Rn]);
            } else {
                sprintf(buffer, "BX%s [r%u] =%Xh ARM",
                conditions_extensions[inst.Cond],
                 inst.branch_Rn, rf[inst.branch_Rn]);
            }
            break;
        }
        case ldst: {
            sprintf(buffer, "%s%s%s%s r%u, [r%u, %dh] =%08Xh",
                instruction_stems[mnemonic],
                conditions_extensions[inst.Cond],
                (inst.ldst_B && (mnemonic == LDR || mnemonic == STR)) ? "B" : "",
                (inst.ldst_P) ? "" : "T",
                inst.ldst_Rd,
                inst.ldst_Rn,
                (inst.ldst_U) ? 
                get_mem_address(inst, instruction_addr, true) :
                    -1 * get_mem_address(inst, instruction_addr, true),
                get_mem_address(inst, instruction_addr)
            );    
            break;
        }
        case NOOP: {
            sprintf(buffer, "NOOP"); 
            break;
        }
    }

}

void Arm::execute(uint32_t instruction, InstructionMnemonic mnemonic){

    //need to look at flags
    if(!condition_pass((ConditionField)CONDITION(instruction))) return;

    InstructionAttributes inst;
    inst.word = instruction;

    switch(mnemonic){
        case B: {
            //why am I 4 off? I guess, read the thing
            if(inst.branch_link) rf[LR] = rf[PC];
            int32_t offset = inst.branch_offset << 2;
            rf[PC] = (uint32_t)(rf[PC] + offset + 4);
            break;
        }
        case BX: {
            //Swap Modes
            rf[PC] = rf[inst.branch_Rn]; //first register bits[3:0]
            break;
        }
        case LDRH: case STRH: 
        case LDRSB: case LDRSH:
        case LDR: case STR: {

            MemOp transfer_instruction;
            uint32_t addr = get_mem_address(inst, rf[PC] - 0x4);
            
            OpType op = (OpType)((inst.ldst_L << 1) | inst.ldst_B);
            
            switch(mnemonic){
                case LDRH: op = ldh; break; 
                case STRH: op = strh; break;
                case LDRSB: op = ldsb; break;
                case LDRSH: op = ldsh; break;
            }

            transfer_instruction.operation = op;
            transfer_instruction.data = rf[inst.ldst_Rd]; //for stor operation

            if(inst.ldst_W && inst.ldst_P) { //may not be functioning correctly
                //pre
                rf[inst.ldst_Rn] = addr;
                transfer_instruction.addr = rf[inst.ldst_Rn];
            } else {
                transfer_instruction.addr = rf[inst.ldst_Rn];
            }

            uint32_t data = MMU(transfer_instruction);

            if(!inst.ldst_P) { //may not be functioning correctly
                rf[inst.ldst_Rn] = addr;
            } 
            
            if(mnemonic != STR || mnemonic != STRH){
                rf[inst.ldst_Rd] = data;
            }
            break;
        }
        //Data Processing
        case MRS: {
            //only cpsr
            rf[inst.dp_Rd] = rf[CPSR];
            break;
        }
        case MSR: { //there is a slight differnce in the docs considering bit16!!
            if(!inst.dp_I && inst.dp_Shift == 0){
                rf[CPSR] = rf[inst.dp_Rm];
                break;
            }
            rf[CPSR] = get_operand_2(inst);
            break;
        }
        case MOV: {
            rf[inst.dp_Rd] = get_operand_2(inst);
            break;
        }
        case MVN: rf[inst.dp_Rd] = 0xFFFFFFFF ^ get_operand_2(inst); break;
        case AND: rf[inst.dp_Rd] = rf[inst.dp_Rn] & get_operand_2(inst); break;
        case EOR: rf[inst.dp_Rd] = rf[inst.dp_Rn] | get_operand_2(inst); break;
        case SUB: rf[inst.dp_Rd] = rf[inst.dp_Rn] - get_operand_2(inst); break;
        case RSB: rf[inst.dp_Rd] = get_operand_2(inst) - rf[inst.dp_Rn]; break;
        case ADD: rf[inst.dp_Rd] = rf[inst.dp_Rn] + get_operand_2(inst); break;
        case ADC: rf[inst.dp_Rd] = rf[inst.dp_Rn] + get_operand_2(inst) + TESTBIT(rf[CPSR], BIT_C); break;
        case SBC: rf[inst.dp_Rd] = rf[inst.dp_Rn] - get_operand_2(inst) + TESTBIT(rf[CPSR], BIT_C); break;
        case RSC: rf[inst.dp_Rd] = get_operand_2(inst) - rf[inst.dp_Rn] + TESTBIT(rf[CPSR], BIT_C); break;
        case BIC: rf[inst.dp_Rd] = rf[inst.dp_Rn] & ~get_operand_2(inst); break;
        case ORR: rf[inst.dp_Rd] = rf[inst.dp_Rn] | get_operand_2(inst); break;
        case UNIMP: {
            break;
        }
    }

}

void Arm::step(){
    uint32_t instruction = fetch();
    InstructionMnemonic instruction_type = decode(instruction);
    execute(instruction, instruction_type);
}

void Arm::register_dump (char* buffer) const{
    //make sure to free afterwards!
    char* dump_ptr = buffer;
    for(uint8_t reg = 0; reg < REGSIZE; reg++){
        if(reg < 10){
            dump_ptr += sprintf(dump_ptr, "r%d:  0x%08X\n", reg, rf[reg]);
        } else if(reg == 16){
            dump_ptr += sprintf(dump_ptr, "cpsr: 0x%08X\n", rf[reg]);
        } else {
            dump_ptr += sprintf(dump_ptr, "r%d: 0x%08X\n", reg, rf[reg]);
        } 
    }
}

void Arm::rom_dump(char* buffer){
    uint32_t pc = rf[PC];
    for (int word = -15; word < 15; word++){
        if((int32_t)(pc + (word*4)) < 0) continue;

        char mnemonic[15] {0};
        Arm::MemOp fetch_operation = {pc + (word*4), Arm::ldw};
        uint32_t instruction = MMU(fetch_operation);

        if(!(pc + (word*4) > 0x8000000 && pc + (word*4) < 0x80000C0)) {
            disassemble(mnemonic, instruction, (pc + (word*4))); 
        } else {
            //Cartidge Header        rf[inst.rd] = result;
            *mnemonic = 'd';
            *(mnemonic + 1) = 'd'; 
        } 

        if (word == 0) {
            buffer += sprintf(buffer, "0x%08X: 0x%08X  %s <-\n", pc + (4*word), instruction, mnemonic);
        } else {
            buffer += sprintf(buffer, "0x%08X: 0x%08X  %s\n", pc + (4*word), instruction, mnemonic);
        }
    }
}

void Arm::set_MMU(void* func_ptr){
    MMU = (uint32_t(*)(MemOp)) func_ptr;
}

uint32_t Arm::get_pc() const{
    return rf[PC];
}

uint32_t Arm::get_mem_address(InstructionAttributes inst, uint32_t instruction_addr, bool offset_only){
    uint32_t addr_offset = 0;
    /*
        Write-back must not be specified if R15 is specified as the base register (Rn). When
        using R15 as the base register you must remember it contains an address 8 bytes on
        from the address of the current instruction.
        R15 must not be specified as the register offset (Rm).
        When R15 is the source register (Rd) of a register store (STR) instruction, the stored
        value will be address of the instruction plus 12.
    */
    // assert(!(inst.ldst_W && inst.ldst_Rn == PC));
    // assert(inst.ldst_Rm != PC);

    if(inst.ldst_I){
        inst.ldst_I = 0;
        addr_offset = get_operand_2(inst);
        inst.ldst_I = 1;
    } else {
        addr_offset = inst.ldst_Imm;
    }

    switch(decode(inst.word)){
        case LDRH: case STRH: case LDRSB: case LDRSH: {
            if(!inst.ldstx_immediate_type && inst.ldstx_offset_2 == 0) {
                //register offset
                addr_offset = rf[inst.ldstx_Rm];
            } else if (inst.ldstx_immediate_type) {
                //immediate offset
                addr_offset = (inst.ldstx_offset_2 << 4) | inst.ldstx_offset_1;
            }
        }
    }

    if(offset_only) return addr_offset;

    uint32_t base = (inst.ldst_Rn == PC) ? instruction_addr + 0x8 : rf[inst.ldst_Rn]; //8 for pipeline waiting
    uint32_t addr = (inst.ldst_U) ? base + addr_offset : base - addr_offset;
    return addr;
}

uint32_t Arm::get_operand_2(InstructionAttributes inst){

    PSR cpsr;
    cpsr.double_word = rf[CPSR];
    uint8_t carry_out = cpsr.C;

    uint32_t result = 0;

    //I don't know which instructions are RRX(extended) and RR0
    if(inst.dp_I){
        //rotated immediate operand 2 
        uint32_t imm = inst.dp_Imm;
        uint8_t rotations = inst.dp_Rotate * 2;
        for (uint8_t times = 0; times < rotations; times++){
            carry_out = imm & 0x1;
            imm >>= 1;
            imm |= (carry_out << 31);
        }
        result = imm;

    } else {

        //shifted/rotated register
        uint32_t reg_value = rf[inst.dp_Rm];
        // printf("%X\n", reg_value);
        uint8_t shifts;
        if(inst.dp_reg_shift && !inst.dp_reg_padding_bit){
            //register shift

            shifts = (uint8_t)(rf[inst.dp_Rs] & 0xFF);
        } else if(!inst.dp_reg_shift){
            shifts = inst.dp_shift_amount; //bit4 not set
            // printf("shifts:%d", shifts);
        } else {
            // printf("Incorrect Configurations for Register Shifting!\n");
            return 0;
        }

        if(shifts == 0) return reg_value;

        switch(inst.dp_shift_type){
            case 0B00:{ //sll
                // printf("here");
                reg_value <<= shifts - 1;
                carry_out = (reg_value & 0x80000000) >> 31;
                result = reg_value << 1;
                break;
            }
            case 0B01:{ //slr
                reg_value >>= shifts - 1;
                carry_out = (reg_value & 0x1);
                result = reg_value >> 1;
                break;
            }
            case 0B10:{ //sar
                result = (uint32_t)(((int32_t)reg_value) >> shifts);
                if (shifts < 32) {
                    carry_out = (reg_value >> shifts - 1) & 0x1;
                } else carry_out = reg_value >> 31;
                break;
            }
            case 0B11:{ //rr
                for (uint8_t times = 0; times < shifts; times++){
                    carry_out = reg_value & 0x1;
                    reg_value >>= 1;
                    reg_value |= (carry_out << 31);
                }
                result = reg_value;
                break;
            }
            default:{ //impossible
                result = 0;
                break;
            }
        }
    }

    /*
        The arithmetic operations (SUB, RSB, ADD, ADC, SBC, RSC, CMP, CMN) treat each
        operand as a 32 bit integer (either unsigned or 2's complement signed, the two are
        equivalent). If the S bit is set (and Rd is not R15) the V flag in the CPSR will be set if
        an overflow occurs into bit 31 of the result; this may be ignored if the operands were
        considered unsigned, but warns of a possible error if the operands were 2's
        complement signed. The C flag will be set to the carry out of bit 31 of the ALU, the Z
        flag will be set if and only if the result was zero, and the N flag will be set to the value
        of bit 31 of the result (indicating a negative result if the operands are considered to be
        2's complement signed).
    */

    if(inst.dp_S && inst.dp_Rd != 15) {
        //set status flags
        cpsr.C = carry_out;
        //V ? 
        if (result & (0x1 << 31)) cpsr.V = 1;
        cpsr.Z = (result == 0) ? 1 : 0;
        cpsr.N = (result & (0x1 << 31)) ? 1 : 0;
        rf[CPSR] = cpsr.double_word;
    }
    return result;
}

void Arm::dump_instructions_attributes(InstructionAttributes inst, DisplayTypes type){
    switch(type){
        case dp_1: case dp_2: case dp_3: {
            printf("Instruction: %08X\n", inst.word);
            printf("operand_2: 0x%03X\n", inst.dp_operand_2);
            printf("Rd: 0x%0X\n", inst.dp_Rd);
            printf("Rn: 0x%0X\n", inst.dp_Rn);
            printf("S: %X\n", inst.dp_S);
            printf("OpCode: 0x%0X\n", inst.dp_OpCode);
            printf("I: %X\n", inst.dp_I);
            printf("Cond: 0x%0X\n", inst.Cond);
        }
    }
}

Arm::DisplayTypes Arm::get_display_type(InstructionMnemonic mnemonic){
    switch(mnemonic){
        case MOV: case MVN: case MRS: case MSR:{
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

        default:
            return NOOP;
    }
}