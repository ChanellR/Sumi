#include "../include/Sumi/arm.hpp"

#include <cstdio>
#include <iostream>
#include <string>
#include <stdint.h>
#include <assert.h>
#include <cstring>
#include <tuple>

using namespace ARM;

void ARMCore::Reset(){
    //sets all registers to 0
    memset(rf, 0, 4*17);
    //for now
    rf[PC] = 0x08000000; //beginning of ROM
    rf[LR] = 0x08000000; //beginning of ROM
    rf[SP] = 0x03007F00;
    rf[CPSR] = 0x0000001F; //user mode
}


void ARMCore::SetMMU(void* func_ptr){
    MMU = (uint32_t(*)(MemOp)) func_ptr;
}

void ARMCore::SetReg(uint16_t reg, uint32_t value){
    rf[reg] = value;
}

uint32_t ARMCore::GetReg(uint16_t reg) const{
    return rf[reg];
}

inline State ARMCore::GetOperatingState() const {
    PSR cpsr {rf[CPSR]};
    return State(cpsr.operating_state); //1: Thumb 0: Arm
}


uint32_t ARMCore::Fetch(State current_state){
    MemOp fetch_operation {rf[PC], (current_state) ? ldh : ldw, 0};
    uint32_t instruction = MMU(fetch_operation);
    rf[PC] += (current_state) ? 2 : 4;
    return instruction;
}


std::pair<InstructionMnemonic, uint16_t> ARMCore::Decode(uint32_t instruction) const{

    const InstructionAttributes attributes {instruction};


    //TODO;
    //MUL, Coprocessor

    if(attributes.ldstM_valid == 0B100) return std::make_pair((attributes.ldstM_L) ? LDM : STM, 0);
    if(attributes.ldst_identifier == 0B01) return std::make_pair((attributes.ldst_L) ? LDR : STR, 0);
    
    if(MATCH(instruction & 0xe0000010, 0x60000010)) return std::make_pair(UNDEF, 0);
    if(MATCH(instruction & 0x0ffffff0, 0x012FFF10)) return std::make_pair(BX, 0);
    if(MATCH(instruction & 0x0F000000, 0x0F000000)) return std::make_pair(SWI, 0);
    if(MATCH(instruction & 0x0E000000, 0x0A000000)) return std::make_pair(B, 0);

    //Halfword and Signed Data Transfer && SWP && MUL
    if (attributes.ldstx_valid_3 == 0 && attributes.ldstx_valid_2 && attributes.ldstx_valid_1) {
        if(attributes.ldstx_SH == 0){
            //MUL && SWP
            if(MATCH(instruction & 0x0FB00FF0, 0x01000090)){
                return std::make_pair(SWP, 0);
            }
            if(attributes.mul_identifier_2 == 0){
                if(attributes.mul_A) {
                    return std::make_pair(MUL, 0);
                } else return std::make_pair(MLA, 0);
            }
            if(attributes.mull_identifier_2 == 1){
                if(attributes.mull_A) {
                    return std::make_pair(MULL, 0);
                } else return std::make_pair(MLAL, 0);
            }

        } else if(attributes.ldstx_L){
            //load
            if(!attributes.ldstx_immediate_type && attributes.ldstx_offset_2 == 0) {
                //register offset
                switch(attributes.ldstx_SH){
                    case 0B01: return std::make_pair(LDRH, 0);
                    case 0B10: return std::make_pair(LDRSB, 0); 
                    case 0B11: return std::make_pair(LDRSH, 0);
                }
            } else if (attributes.ldstx_immediate_type) {
                //immediate offset
                switch(attributes.ldstx_SH){
                    case 0B01: return std::make_pair(LDRH, 0);
                    case 0B10: return std::make_pair(LDRSB, 0); 
                    case 0B11: return std::make_pair(LDRSH, 0);
                }
            }

        } else {
            //store
            if(!attributes.ldstx_immediate_type && attributes.ldstx_offset_2 == 0) {
                //register offset
                return std::make_pair(STRH, 0);
            } else if (attributes.ldstx_immediate_type) {
                //immediate offset
                return std::make_pair(STRH, 0);
            }
        }
    }

    if(attributes.dp_valid == 0){
        //checks for Immediate offset or register with proper_validation_bits
        //0B1110_00_1_1010_1_0010_0000_0000_00111001 
        // printf("Imm_type: %u\n", attributes.dp_I);
        if(attributes.dp_I || ((!attributes.dp_reg_padding_bit && attributes.dp_reg_shift) || !attributes.dp_reg_shift)){
            InstructionMnemonic prelim_type = (InstructionMnemonic)attributes.dp_OpCode;
            // printf("prelim_type: %u\n", prelim_type);
            // printf("S: %d\n", attributes.dp_S);
            // 0xE128F00A
            //0B1110_00_0_10_0_1010001111_00000000_1010
            if(prelim_type == TEQ || prelim_type == TST || prelim_type == CMN || prelim_type == CMP){
                if(!attributes.dp_S){
                    if(MATCH(instruction & 0x0FBF0FFF, 0x010F0000)){
                        //transfer PSR contents to a register
                        return std::make_pair(MRS, 0);
                    } else if (MATCH(instruction & 0x0FBFFFF0, 0x0129F000)){
                        //transfer register contents to PSR
                        return std::make_pair(MSR, 0);
                    } else if (MATCH(instruction & 0x0DBFF000, 0x0128F000)){
                        //transfer register contents or immediate value 
                        //to flag bits only
                        return std::make_pair(MSRf, 0);
                    } 
                } 

            } 
            return std::make_pair(prelim_type, 0);
        }
    }

    return std::make_pair(UNIMP, 0);
}


std::pair<InstructionMnemonic, uint16_t> ARMCore::Decode_T(uint16_t instruction) const{
    const InstructionAttributes attributes {instruction};

    if(attributes.t_f1_indentifier == 0B000){ //Register Offset
        if(attributes.t_f1_Opcode == 0B11){ //add/subtract
            return std::make_pair((attributes.t_f2_Opcode == 0) ? ADD : SUB, 2);
        }
        return std::make_pair(MOV, 1);
    }

    if(attributes.t_f3_indentifier == 0B001){ //Immediate Offset
        switch(attributes.t_f3_Opcode){
            case 0B00: return std::make_pair(MOV, 3);
            case 0B01: return std::make_pair(CMP, 3);
            case 0B10: return std::make_pair(ADD, 3);
            case 0B11: return std::make_pair(SUB, 3);
        }
    }

    if(attributes.t_f4_identifier == 0B010000){
        switch(attributes.t_f4_Opcode){
            case 0B0000: return std::make_pair(AND, 4);
            case 0B0001: return std::make_pair(EOR, 4);
            case 0B0010: return std::make_pair(MOV, 4);
            case 0B0011: return std::make_pair(MOV, 4);
            case 0B0100: return std::make_pair(MOV, 4);
            case 0B0101: return std::make_pair(ADC, 4);
            case 0B0110: return std::make_pair(SBC, 4);
            case 0B0111: return std::make_pair(MOV, 4);
            case 0B1000: return std::make_pair(TST, 4);
            case 0B1001: return std::make_pair(RSB, 4);
            case 0B1010: return std::make_pair(CMP, 4);
            case 0B1011: return std::make_pair(CMN, 4);
            case 0B1100: return std::make_pair(ORR, 4);
            case 0B1101: return std::make_pair(MUL, 4);
            case 0B1110: return std::make_pair(BIC, 4);
            case 0B1111: return std::make_pair(MVN, 4);
        }
    }

    if(attributes.t_f5_identifier == 0B010001){
        switch(attributes.t_f5_Opcode){
            case 0B00: return std::make_pair(ADD, 5);
            case 0B01: return std::make_pair(CMP, 5);
            case 0B10: return std::make_pair(MOV, 5);
            case 0B11: if(attributes.t_H1 == 0) return std::make_pair(BX, 5);
        }
    }

    if(attributes.t_f6_identifier == 0B01001) return std::make_pair(LDR, 6);

    if(attributes.t_f7_identifier2 == 0B0101 && attributes.t_f7_identifier1 == 0B0) {
        switch(attributes.t_L << 1 | attributes.t_f7_B){
            case 0B00: return std::make_pair(STR, 7);
            case 0B01: return std::make_pair(STR, 7); //byte
            case 0B10: return std::make_pair(LDR, 7);
            case 0B11: return std::make_pair(LDR, 7);
        } //fine
    }

    if(attributes.t_f8_identifier2 == 0B0101 && attributes.t_f8_identifier1 == 0B1) { //register offset
        switch(attributes.t_f8_S << 1 | attributes.t_H){
            case 0B00: return std::make_pair(STR, 8);
            case 0B01: return std::make_pair(LDRH, 8);
            case 0B10: return std::make_pair(LDRSB, 8);
            case 0B11: return std::make_pair(LDRSH, 8);
        } //fine
    }

    if(attributes.t_f910_identifier == 0B011){ //immediate offset
        switch(attributes.t_L << 1 | attributes.t_f910_B){
            case 0B00: return std::make_pair(STR, 9);
            case 0B01: return std::make_pair(LDR, 9); 
            case 0B10: return std::make_pair(STR, 9);
            case 0B11: return std::make_pair(LDR, 9); //byte immediate
        } //fine
    }

    if(attributes.t_f910_identifier == 0B100 && attributes.t_f910_B == 0B0){ //immediate offset
        switch(attributes.t_L){
            case 0B00: return std::make_pair(STRH, 10);
            case 0B01: return std::make_pair(LDRH, 10); 
        } //fine
    }

    if(attributes.t_f11_identifier == 0B1001){ //SP-relative 
        switch(attributes.t_L){
            case 0B00: return std::make_pair(STR, 11);
            case 0B01: return std::make_pair(LDR, 11); 
        } //fine
    }
    
    if(attributes.t_f12_identifier == 0B1010){
        return std::make_pair(ADD, 12);
    }

    if(attributes.t_f13_identifier == 0B10110000){
        return std::make_pair((attributes.t_f13_S) ? SUB : ADD, 13);
    }

    if(attributes.t_f14_identifier2 == 0B1011 && attributes.t_f14_identifier1 == 0B10){ //immediate offset
        switch(attributes.t_L << 1 | attributes.t_f14_R){
            case 0B00: return std::make_pair(STM, 14);
            case 0B01: return std::make_pair(STM, 14); 
            case 0B10: return std::make_pair(LDM, 14);
            case 0B11: return std::make_pair(LDM, 14); //byte immediate
        } //fine
    }

    if(attributes.t_f15_identifier == 0B1100){
        return std::make_pair((attributes.t_L) ? LDM : STM, 15);
    }

    if(attributes.t_f16_identifier == 0B1101) return std::make_pair(B, 16);

    if(attributes.t_f17_identifier == 0B11011111) return std::make_pair(SWI, 17);

    if(attributes.t_f18_identifier == 0B11100) return std::make_pair(B, 18);

    if(attributes.t_f19_identifier == 0B1111) return std::make_pair(B, 19);

    return std::make_pair(UNIMP, 0);
}


bool ARMCore::Condition(uint8_t condition) const{
    PSR cpsr {rf[CPSR]};
    bool outcome = false;
    switch (condition)
    {
        case EQ: outcome = cpsr.Z; break;
        case NE: outcome = !cpsr.Z;  break;
        case CS: outcome = cpsr.C;  break;
        case CC: outcome = !cpsr.C;  break;
        case MI: outcome = cpsr.N;  break;
        case PL: outcome = !cpsr.N;  break;
        case VS: outcome = cpsr.V;  break;
        case VC: outcome = !cpsr.V;  break;
        case HI: outcome = cpsr.C && !cpsr.Z;  break;
        case LS: outcome = !cpsr.C || cpsr.Z;  break;
        case GE: outcome = cpsr.N == cpsr.V;  break;
        case LT: outcome = cpsr.N != cpsr.V;  break;
        case GT: outcome = !cpsr.Z && (cpsr.N == cpsr.V);  break;
        case LE: outcome = cpsr.Z || (cpsr.N != cpsr.V);  break;
        case AL: outcome = true;  break;
    }
    return outcome;
}


void ARMCore::Execute(Instruction inst){

    //Check Condition
    if(!Condition(inst.attributes.Cond)) return;
    
    //Most data processes follow the same flag 
    //setting criteria when activated
    //will seperate into similar sections

    switch (inst.mnemonic)
    {
        case AND: case EOR: case SUB: case RSB: 
        case ADD: case ADC: case SBC: case RSC: 
        case TST: case TEQ: case CMP: case CMN: 
        case ORR: case MOV: case BIC: case MVN:
        case MRS: case MSR: case MSRf: {
            return DataProcessingInstruction(inst);  
        }
        case B: case BX: {
            return BranchInstruction(inst);
        }
        case LDRH: case STRH: case LDRSB: case LDRSH: 
        case LDM: case STM: case LDR: case STR: case SWP: {
            return DataTransferInstruction(inst);
        }
        case MUL: case MLA: case MULL: case MLAL: {
            return MultiplyInstruction(inst);
        }
        case UNDEF: case UNIMP:{
            return;
        }
    }
}


void ARMCore::Step(){

    State current_state = GetOperatingState();
    uint32_t instruction = Fetch(current_state);

    uint16_t format;
    InstructionMnemonic mnem;
    std::tie(mnem, format) = (current_state) ? Decode_T(instruction) : Decode(instruction);
    Instruction inst {mnem, instruction, rf[PC] - ((current_state) ? 2 : 4), format};

    Execute(inst);
}


uint32_t ARMCore::GetMemAddress(Instruction inst, bool offset_only) const{
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

    if(inst.attributes.ldst_I){
        inst.attributes.ldst_I = 0;
        std::pair<uint32_t, uint8_t> out;
        out = GetOperand(inst);
        addr_offset = out.second;
        inst.attributes.ldst_I = 1;
    } else {
        addr_offset = inst.attributes.ldst_Imm;
    }

    switch(inst.mnemonic){
        case LDRH: case STRH: case LDRSB: case LDRSH: {
            if(!inst.attributes.ldstx_immediate_type && inst.attributes.ldstx_offset_2 == 0) {
                //register offset
                addr_offset = rf[inst.attributes.ldstx_Rm];
            } else if (inst.attributes.ldstx_immediate_type) {
                //immediate offset
                addr_offset = (inst.attributes.ldstx_offset_2 << 4) | inst.attributes.ldstx_offset_1;
            }
        }
    }

    if(offset_only) return addr_offset;

    uint32_t base = (inst.attributes.ldst_Rn == PC) ? inst.address + 0x8 : rf[inst.attributes.ldst_Rn]; //8 for pipeline waiting
    uint32_t addr = (inst.attributes.ldst_U) ? base + addr_offset : base - addr_offset;
    return addr;
}


std::pair<uint32_t, uint8_t> ARMCore::GetOperand(Instruction inst) const {
    //Derives Operand 2 for Data Processing Instructions

    PSR cpsr {rf[CPSR]};

    uint8_t carry_out = cpsr.C;
    uint32_t result = 0;

    if(inst.attributes.dp_I){
        //rotated immediate operand 2 
        uint32_t imm = inst.attributes.dp_Imm;
        uint8_t rotations = inst.attributes.dp_Rotate * 2;
        for (uint8_t times = 0; times < rotations; times++){
            carry_out = imm & 0x1;
            imm >>= 1;
            imm |= (carry_out << 31);
        }
        result = imm;

    } else {

        uint32_t reg_value = rf[inst.attributes.dp_Rm];
        uint16_t shifts;

        if(inst.attributes.dp_reg_shift && !inst.attributes.dp_reg_padding_bit){
            shifts = rf[inst.attributes.dp_Rs];
        } else if(!inst.attributes.dp_reg_shift){
            shifts = inst.attributes.dp_shift_amount;
        } 

        if(shifts == 0) return std::make_pair(reg_value, carry_out);

        switch(inst.attributes.dp_shift_type){
            case 0B00:{ //lsl
                if(shifts > 31) {
                    result = 0;
                    carry_out = (shifts == 32) ? TESTBIT(reg_value, 0) : 0;
                    break;
                }
                reg_value <<= (shifts - 1);
                carry_out = (reg_value & 0x80000000) >> 31;
                result = reg_value << 1;
                break;
            }
            case 0B01:{ //lsr
                if(shifts > 31) {
                    result = 0;
                    carry_out = (shifts == 32) ? TESTBIT(reg_value, 31) : 0;
                    break;
                }
                reg_value >>= shifts - 1;
                carry_out = (reg_value & 0x1);
                result = reg_value >> 1;
                break;
            }
            case 0B10:{ //asr
                if (shifts < 32) {
                    result = (uint32_t)(((int32_t)reg_value) >> shifts);
                    carry_out = (reg_value >> shifts - 1) & 0x1;
                } else {
                    carry_out = (reg_value & 0x80000000) ? 1 : 0;
                    result = (reg_value & 0x80000000) ? -1 : 0;
                }
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
        }
    }

    return std::make_pair(result, carry_out);
    
}


void ARMCore::MultiplyInstruction(Instruction inst){
    switch (inst.mnemonic)
    { 
        case MUL: case MLA: {
            uint32_t mul_result = rf[inst.attributes.mul_Rm] * rf[inst.attributes.mul_Rs];
            if(inst.attributes.mull_A) mul_result += rf[inst.attributes.mul_Rn];
            rf[inst.attributes.mul_Rd] = mul_result;
            if(inst.attributes.mul_S){
                PSR cpsr {rf[CPSR]};
                cpsr.N = (mul_result & (0x1 << 31)) ? 1 : 0;
                cpsr.Z = (mul_result == 0);
                rf[CPSR] = cpsr.word;
            }  
            return;
        }
        case MULL: case MLAL: {
            int64_t mul_result = 0;

            if(inst.attributes.mull_U){
                //signed
                mul_result = (int64_t)(int32_t)rf[inst.attributes.mull_Rm] * (int64_t)(int32_t)rf[inst.attributes.mull_Rs];
                // if(rf[inst.attributes.mull_Rm] == 0x0DD37FB9) printf("RES: %X\n", mul_result);
                if(inst.attributes.mull_A) mul_result += ((int64_t)(int32_t)rf[inst.attributes.mull_RdHi]) << 32 | (int64_t)(int32_t)rf[inst.attributes.mull_RdLo];
                // printf("resHi: %X resLo: %X Op1: %X Op2: %X\n", res >> 32, res, Rm, Rs);
                rf[inst.attributes.mull_RdHi] = mul_result >> 32;
            } else {
                //unsigned
                mul_result = (uint64_t)rf[inst.attributes.mull_Rm] * (uint64_t)rf[inst.attributes.mull_Rs];
                if(inst.attributes.mull_A) mul_result +=  (uint64_t)rf[inst.attributes.mull_RdHi] << 32 | (uint64_t)rf[inst.attributes.mull_RdLo];
                rf[inst.attributes.mull_RdHi] = mul_result >> 32;
            }
            
            rf[inst.attributes.mull_RdLo] = (uint32_t)mul_result;

            if(inst.attributes.mull_S){
                PSR cpsr {rf[CPSR]};
                cpsr.N = (mul_result & ((uint64_t)0x1 << 63)) ? 1 : 0;
                cpsr.Z = (mul_result == 0);
                rf[CPSR] = cpsr.word;
            }   
            return;
        }
    }
}


void ARMCore::BranchInstruction(Instruction inst){
    switch(inst.mnemonic){
        case B: {
            //why am I 4 off? I guess, read the thing -> PIPELINE!
            //I already step so I don't need to add another 4
            if(inst.attributes.branch_link) rf[LR] = rf[PC];
            int32_t offset = inst.attributes.branch_offset << 2;
            rf[PC] = (uint32_t)(rf[PC] + offset + 4);
            break;
        }
        case BX: {
            //Swap Modes
            PSR cpsr {rf[CPSR]};
            cpsr.operating_state = rf[inst.attributes.branch_Rn] & 0x1;
            rf[CPSR] = cpsr.word;
            rf[PC] = rf[inst.attributes.branch_Rn]; //first register bits[3:0]
            break;
        }
    }
}


void ARMCore::DataTransferInstruction(Instruction inst){
  //Memory Transfers
    switch(inst.mnemonic){
        case LDRH: case STRH: case LDRSB: case LDRSH: case LDR: case STR: {
            MemOp transfer_instruction;
            uint32_t addr = GetMemAddress(inst);
            
            OpType op = (OpType)((inst.attributes.ldst_L << 1) | inst.attributes.ldst_B);
            
            switch(inst.mnemonic){
                case LDRH: op = ldh; break; 
                case STRH: op = strh; break;
                case LDRSB: op = ldsb; break;
                case LDRSH: op = ldsh; break;
            }

            transfer_instruction.operation = op;
            transfer_instruction.data = rf[inst.attributes.ldst_Rd]; //for stor operation

            if(inst.attributes.ldst_W && inst.attributes.ldst_P) { //may not be functioning correctly
                //pre
                rf[inst.attributes.ldst_Rn] = addr;
                transfer_instruction.addr = rf[inst.attributes.ldst_Rn];
            } else {
                transfer_instruction.addr = rf[inst.attributes.ldst_Rn];
            }

            uint32_t data = MMU(transfer_instruction);

            if(!inst.attributes.ldst_P) {
                rf[inst.attributes.ldst_Rn] = addr;
            } 
            
            if(inst.mnemonic != STR && inst.mnemonic != STRH){
                rf[inst.attributes.ldst_Rd] = data;
            }
            return;
        }
        case LDM: case STM: {
            //what registers
            uint32_t base = rf[inst.attributes.ldstM_Rn];
            MemOp transfer {base, (inst.attributes.ldstM_L) ? ldw : strw, 0};
            if(inst.attributes.ldstM_U){
                for(int reg = 0; reg < 16; reg++){
                    if(TESTBIT(inst.attributes.ldstM_register_list, reg)){
                        //transfer from the where Op1 is pointing for this register
                        transfer.data = rf[reg];
                        if(inst.attributes.ldstM_P){
                            transfer.addr += 0x4;
                            uint32_t out = MMU(transfer);
                            if(inst.attributes.ldstM_L) rf[reg] = out;
                        } else {
                            uint32_t out = MMU(transfer);
                            if(inst.attributes.ldstM_L) rf[reg] = out;
                            transfer.addr += 0x4;
                        }
                    }
                }
                if(inst.attributes.ldstM_W) rf[inst.attributes.ldstM_Rn] = transfer.addr;
            } else {
                for(int reg = 15; reg >= 0; reg--){
                    if(TESTBIT(inst.attributes.ldstM_register_list, reg)){
                        //transfer from the where Rn is pointing for this register
                        transfer.data = rf[reg];
                        if(inst.attributes.ldstM_P){
                            transfer.addr -= 0x4;
                            uint32_t out = MMU(transfer);
                            if(inst.attributes.ldstM_L) rf[reg] = out;
                        } else {
                            uint32_t out = MMU(transfer);
                            if(inst.attributes.ldstM_L) rf[reg] = out;
                            transfer.addr -= 0x4;
                        }
                    }
                }
                if(inst.attributes.ldstM_W) rf[inst.attributes.ldstM_Rn] = transfer.addr;
            }
            return;
        }
        case SWP:{
            MemOp load {rf[inst.attributes.ldst_Rn], (inst.attributes.ldst_B) ? ldb : ldw, 0};
            MemOp store {rf[inst.attributes.ldst_Rn], (inst.attributes.ldst_B) ? strb : strw, rf[inst.attributes.ldst_Rm]};
            rf[inst.attributes.ldst_Rd] = MMU(load);
            MMU(store);
            return;
        }
    }
}


void ARMCore::DataProcessingInstruction(Instruction inst){
    
    uint32_t result;
    uint32_t Op1, Op2;
    uint8_t C, Rd;
  

    if(GetOperatingState()) {
        //go through a different process of getting Operands
        switch(inst.format){
            case 1: {
                
                break;
            }
        }

    } else {
        Op1 = rf[inst.attributes.dp_Rn];
        Rd = inst.attributes.dp_Rd;
        std::tie(Op2, C) = GetOperand(inst);
    }

    InstructionMnemonic mnemonic = inst.mnemonic;
    
    uint8_t old_carry = C;

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
        case MRS: rf[inst.attributes.dp_Rd] = rf[CPSR]; return;
        case MSR: case MSRf: { //there is a slight differnce in the docs considering bit16!!
            uint32_t new_flag_reg;

            if(!inst.attributes.dp_I && inst.attributes.dp_Shift == 0){
                new_flag_reg = rf[inst.attributes.dp_Rm];
            } else {
                new_flag_reg = C;
            }

            if(mnemonic == MSRf){
                rf[CPSR] &= ~(0xF0000000);
                rf[CPSR] |= new_flag_reg & (0xF0000000);
            } else {
                rf[CPSR] = new_flag_reg;
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
            result += C; 
            break;
        }
    }

    bool no_write = (inst.mnemonic == TST || inst.mnemonic == TEQ || inst.mnemonic == CMP || inst.mnemonic == CMN);
    
    if(!no_write) {
        rf[Rd] = result;
    }
    
    if(inst.attributes.dp_S && inst.attributes.dp_Rd != 15) { //check for THUMB MODE THINGS
        PSR cpsr {rf[CPSR]};
        if(logical){
            cpsr.C = old_carry;
        } else {
            cpsr.V = (~(Op2 ^ Op1) & (result ^ Op1)) >> 31;
            switch (inst.mnemonic)
            {
                case CMP: case SUB: case SBC: cpsr.C = (Op1 >= ~Op2); break;
                case RSB: case RSC: cpsr.C = (Op2 >= ~Op1); break;    
                default: cpsr.C = ((0xFFFFFFFF - Op1) < Op2); break;
            }
        }
        cpsr.Z = (result == 0) ? 1 : 0;
        cpsr.N = (result & (0x1 << 31)) ? 1 : 0;
        rf[CPSR] = cpsr.word;
    }
}


void ARMCore::RegisterDump (char* buffer) const{
    //make sure to free afterwards!
    char* dump_ptr = buffer;
    for(uint8_t reg = 0; reg < REGSIZE; reg++){
        if(reg < 10){
            dump_ptr += sprintf(dump_ptr, "r%d:  0x%08X\n", reg, rf[reg]);
        } else if(reg == 16){
            dump_ptr += sprintf(dump_ptr, "cpsr: 0x%08X\n", rf[reg]);
        } else if(reg == 17){
            dump_ptr += sprintf(dump_ptr, "spsr: 0x%08X\n", rf[reg]);
        } else {
            dump_ptr += sprintf(dump_ptr, "r%d: 0x%08X\n", reg, rf[reg]);
        } 
    }
}


void ARMCore::RomDump(char* buffer){
    uint32_t pc = rf[PC];
    for (int word = -15; word < 15; word++){
        if((int32_t)(pc + (word*4)) < 0) continue;

        char mnemonic[15] {0};
        MemOp load = {pc + (word*4), ldw};
        uint32_t instruction = MMU(load);

        if(!(pc + (word*4) > 0x8000000 && pc + (word*4) < 0x80000C0)) {
            Disassemble(mnemonic, instruction, pc + (word*4)); 
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


ARM::DisplayTypes ARMCore::GetDisplayType(InstructionMnemonic mnemonic) const{

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


void ARMCore::Info(Instruction inst) const{
    char Disassembly[20];
    DisplayTypes type = GetDisplayType(inst.mnemonic);
    Disassemble(Disassembly, inst.attributes.word, inst.address);
    printf("instruction: 0x%X %s\n",inst.attributes.word, Disassembly);
    printf("mnemonic: %d display type: %d\n", inst.mnemonic, type);
    printf("Imm?: %X type: 0x%X reg?: %u shift_reg:%u shift_amount:0x%X base_reg:%u \n",
    inst.attributes.dp_I, inst.attributes.dp_shift_type, inst.attributes.dp_reg_shift, inst.attributes.dp_Rs, inst.attributes.dp_shift_amount, inst.attributes.dp_Rm);
}


void ARMCore::Disassemble(char* buffer, uint32_t instruction, uint32_t instruction_addr) const{
    //for translating into readable syntax

    InstructionMnemonic mnem;
    uint16_t format;
    std::tie(mnem, format) = (GetOperatingState()) ? Decode_T(instruction) : Decode(instruction);
    Instruction inst {mnem, instruction, instruction_addr, format};
    DisplayTypes type = GetDisplayType(inst.mnemonic);

    if(GetOperatingState()) return;

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

    if(GetOperatingState()){
        //change fields to display correctly
    }

    switch(type){
        case dp_1:{
            uint16_t Rd = inst.attributes.dp_Rd;
            uint16_t Rm = inst.attributes.dp_Rm;
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
                        GetOperand(inst)
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
                        GetOperand(inst)
                        );
                }
            } else {
                sprintf(buffer, "%s%s%s r%u, %Xh",
                    instruction_stems[inst.mnemonic],
                    conditions_extensions[inst.attributes.Cond],
                    (inst.attributes.dp_S) ? "S" : "",
                    Rd, GetOperand(inst));
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
                inst.attributes.dp_Rn, GetOperand(inst));
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
                inst.attributes.dp_Rd, inst.attributes.dp_Rn, GetOperand(inst));
            }
            break;
        }
        case b_1: {
            sprintf(buffer, "B%s%s pc, %0Xh =%0Xh %s", //off by one sometimes, PIPELINE!
                    (inst.attributes.branch_link) ? "L" : "",
                    conditions_extensions[inst.attributes.Cond],
                    inst.attributes.branch_offset,
                    (uint32_t)(inst.address + (inst.attributes.branch_offset << 2) + 8),
                    (Condition(inst.attributes.Cond)) ? "true;" : "false;"
                ); 
            break;
        }
        case b_2:{
            if(rf[inst.attributes.branch_Rn] & 0x1){
                sprintf(buffer, "BX%s [r%u] =%Xh THUMB",
                conditions_extensions[inst.attributes.Cond],
                 inst.attributes.branch_Rn, rf[inst.attributes.branch_Rn]);
            } else {
                sprintf(buffer, "BX%s [r%u] =%Xh ARM",
                conditions_extensions[inst.attributes.Cond],
                 inst.attributes.branch_Rn, rf[inst.attributes.branch_Rn]);
            }
            break;
        }
        case ldst: {
            buffer += sprintf(buffer, "%s%s%s r%u, [r%u, %s%dh]", 
                    instruction_stems[inst.mnemonic],
                    conditions_extensions[inst.attributes.Cond],
                    (inst.attributes.ldst_B && (inst.mnemonic == STR || inst.mnemonic == LDR)) ? "B" : "",
                    inst.attributes.ldst_Rd,
                    inst.attributes.ldst_Rn,
                    (inst.attributes.ldst_U) ? "" : "-",
                    GetMemAddress(inst, true)
                );
            if(rf[PC] <= inst.address + 2) {
                sprintf(buffer, "=%08Xh",
                 (inst.attributes.ldst_P) ? GetMemAddress(inst, true) : rf[inst.attributes.ldst_Rn]);
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
                inst.attributes.ldst_Rn,
                inst.attributes.ldst_Rm
            );
            return;
        }
        case mul: {
            if (inst.mnemonic == MLAL || inst.mnemonic == MULL) {
                sprintf(buffer, "%s%s%s%s r%u, r%u, r%u, r%u",
                        (inst.attributes.mull_U) ? "U" : "",
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

