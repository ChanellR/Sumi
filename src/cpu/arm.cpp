#include <arm.hpp>
#include <bit.hpp>

#include <cstdio>
#include <iostream>
#include <string>
#include <stdint.h>
#include <assert.h>
#include <array>
#include <utility>
#include <tuple>
#include <cstring>

//todo:
//interrupts
//bus signals
//cycle timing
//coprocessor
//timers


namespace ARM {

void Reset(Core& cpu) {
    cpu.rf.Reset();
    Flush(cpu);
    cpu.rf[PC] = 0x08000000; //beginning of ROM
}


// void SetMMU(Core& cpu, void* func_ptr){
//     cpu.MMU = (u32(*)(MemOp)) func_ptr;
// }


void SetReg(Core& cpu, u16 reg, u32 value){
    cpu.rf[reg] = value;
}


u32 GetReg(Core& cpu, u16 reg) {
    return cpu.rf[reg];
}


u32 GetExecuteStageAddr(Core& cpu)  {
    if(cpu.Pipeline.decode_stage.mnemonic != NILL) {
       return cpu.Pipeline.decode_stage.address;
    }
    return 0;
}


inline State GetOperatingState(Core& cpu)  {
    PSR cpsr {cpu.rf[CPSR]};
    return State(cpsr.operating_state); //1: Thumb 0: Arm
}


u32 Fetch(Core& cpu, State current_state){
    MemOp fetch_operation {cpu.rf[PC], (current_state) ? ldh : ldw, 0};
    u32 instruction = cpu.mem.access(fetch_operation);
    cpu.rf[PC] += (current_state) ? 2 : 4;
    return instruction;
}

std::pair<InstructionMnemonic, u16> Decode(u32 instruction) {

    const InstructionAttributes attributes {instruction};

    //TODO;
    //Coprocessor

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
                    return std::make_pair(MLA, 0);
                } else return std::make_pair(MUL, 0);
            }
            if(attributes.mull_identifier_2 == 1){
                if(attributes.mull_A) {
                    return std::make_pair(MLAL, 0);
                } else return std::make_pair(MULL, 0);
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
        if(attributes.dp_I || ((!attributes.dp_reg_padding_bit && attributes.dp_reg_shift) || !attributes.dp_reg_shift)){
            InstructionMnemonic prelim_type = (InstructionMnemonic)attributes.dp_OpCode;
            if(prelim_type == TEQ || prelim_type == TST || prelim_type == CMN || prelim_type == CMP){
                if(!attributes.dp_S && attributes.status_valid2 == 0B10){
                    //MSR
                    if(attributes.status_toPsr && attributes.status_valid1 == 0xF){
                        if((!attributes.dp_I && attributes.status_msr_valid == 0x0) || attributes.dp_I) return std::make_pair(MSR, 0);
                    } 
                    //MRS
                    if(!attributes.status_toPsr && attributes.status_mask == 0xF && attributes.dp_operand_2 == 0x0) {
                        return std::make_pair(MRS, 0);
                    }
                } 

            } 
            return std::make_pair(prelim_type, 0);
        }
    }

    return std::make_pair(UNIMP, 0);
}

constexpr std::pair<InstructionMnemonic, u16> Decode_T(u16 instruction) {
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
            case 0B10: return std::make_pair(LDR, 9); 
            case 0B01: return std::make_pair(STR, 9);
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


void Execute(Core& cpu, Instruction inst){

    //Check Condition
    if(!Condition(cpu, inst.attributes.Cond) && !GetOperatingState(cpu)) return;
    
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
            return DataProcessingInstruction(cpu, inst);  
        }
        case B: case BX: {
            return BranchInstruction(cpu, inst);
        }
        case LDRH: case STRH: case LDRSB: case LDRSH: 
        case LDM: case STM: case LDR: case STR: case SWP: {
            return DataTransferInstruction(cpu, inst);
        }
        case MUL: case MLA: case MULL: case MLAL: {
            return MultiplyInstruction(cpu, inst);
        }
        case SWI:
        case UNDEF: case UNIMP:{
            return;
        }
    }
}


constexpr InstructionMnemonic CDecode(const u32 instruction) {
    //inspired by beeg
    
    constexpr auto data_processing_immediate_shift = bit::decode_hash(0b0000'000'0000'0'0000'0000'00000'00'0'0000);
    constexpr auto data_processing_immediate_shift_mask = bit::decode_hash(0b0000'111'0000'0'0000'0000'00000'00'1'0000);

    constexpr auto data_processing_register_shift = bit::decode_hash(0b0000'000'0000'0'0000'0000'0000'0'00'1'0000);
    constexpr auto data_processing_register_shift_mask = bit::decode_hash(0b0000'111'0000'0'0000'0000'0000'1'00'1'0000);

    //the two bits 22-23 determine long vs short multiplies
    constexpr auto multiply = bit::decode_hash(0b0000'0000'00'00'0000'0000'0000'1001'0000);
    constexpr auto multiply_mask = bit::decode_hash(0b0000'1111'00'00'0000'0000'0000'1111'0000);

    constexpr auto swap = bit::decode_hash(0b0000'0001'00'00'0000'0000'0000'1001'0000);
    constexpr auto swap_mask = bit::decode_hash(0b0000'1111'00'00'0000'0000'0000'1111'0000);

    constexpr auto extended_load_store_register = bit::decode_hash(0b0000'000'00'0'00'0000'0000'0000'1001'0000);
    constexpr auto extended_load_store_register_mask = bit::decode_hash(0b0000'111'00'1'00'0000'0000'1111'1001'0000);

    constexpr auto extended_load_store_immediate = bit::decode_hash(0b0000'000'00'1'00'0000'0000'0000'1001'0000);
    constexpr auto extended_load_store_immediate_mask = bit::decode_hash(0b0000'111'00'1'00'0000'0000'0000'1001'0000);

    //MRS & MSR here
    constexpr auto data_processing_immediate_rotate = bit::decode_hash(0b0000'001'0000'0'0000'0000'0000'00000000);
    constexpr auto data_processing_immediate_rotate_mask = bit::decode_hash(0b0000'111'0000'0'0000'0000'0000'00000000);
    
    if(MATCH(instruction & bit::decode_hash(0x0FF000F0), bit::decode_hash(0x07F000F0))) return UNDEF;

    constexpr auto load_store_immediate = bit::decode_hash(0b0000'011'00000'0000'0000'000000000000);
    constexpr auto load_store_immediate_mask = bit::decode_hash(0b0000'111'00000'0000'0000'000000000000);
 
    constexpr auto load_store_register = bit::decode_hash(0b0000'010'00000'0000'0000'000000000000);
    constexpr auto load_store_register_mask = bit::decode_hash(0b0000'111'00000'0000'0000'000000000000);

    //media instructions

    //architecturally undefined

    constexpr auto load_store_multiple = bit::decode_hash(0b0000'100'00000'0000'0000000000000000);
    constexpr auto load_store_multiple_mask = bit::decode_hash(0b0000'111'00000'0000'0000000000000000);

    constexpr auto branch = bit::decode_hash(0b0000'101'00000'0000'0000'0000'0000'0000);
    constexpr auto branch_mask = bit::decode_hash(0b0000'111'00000'0000'0000'0000'0000'0000);
    if(MATCH(instruction & bit::decode_hash(0x0FFFFFF0), bit::decode_hash(0x12FFF10))) return BX;

    //coprocessor

    constexpr auto software_interrupt = bit::decode_hash(0b0000'11111'000000000000000000000000);
    constexpr auto software_interrupt_mask = bit::decode_hash(0b0000'1111'000000000000000000000000);


    if(MATCH(instruction & data_processing_immediate_shift_mask, data_processing_immediate_shift) \
    || MATCH(instruction & data_processing_register_shift_mask, data_processing_register_shift) \
    || MATCH(instruction & data_processing_immediate_rotate_mask, data_processing_immediate_rotate))
    {
        auto Opcode = bit::get_range<8, 5>(instruction);
        if (Opcode >= 8 && Opcode <= 11) {
            if(!bit::decoded_is_set<20>(instruction)) {
                return (bit::decoded_is_set<21>(instruction)) ? MSR : MRS;
            }
        }
        return InstructionMnemonic(Opcode);
    }

    if(MATCH(instruction & multiply_mask, multiply))
    {
        auto Long = bit::decoded_is_set<23>(instruction);
        auto A = bit::decoded_is_set<21>(instruction);

        if(!Long) {
            if(A){
                return MLA;
            } else {
                return MUL;
            }
        } else {
            if(A){
                return MLAL;
            } else {
                return MULL;
            }
        }
    }

    if(MATCH(instruction & swap_mask, swap)) 
    {
        return SWP;
    }
    
    if(MATCH(instruction & load_store_immediate_mask, load_store_immediate) \
    || MATCH(instruction & load_store_register_mask, load_store_register))
    {
        return (bit::decoded_is_set<20>(instruction)) ? LDR : STR;
    }

    if(MATCH(instruction & extended_load_store_register_mask, extended_load_store_register) \
    || MATCH(instruction & extended_load_store_immediate_mask, extended_load_store_immediate))
    {
        switch(bit::get_range<2, 1>(instruction)){
            case 0b01: {
                return (bit::decoded_is_set<20>(instruction)) ? LDRH : STRH;
            }
            case 0b10: {
                return LDRSB;
            }
            case 0b11: {
                return LDRSH;
            }
        }
    }

    if(MATCH(instruction & load_store_multiple_mask, load_store_multiple))
    {
        return (bit::decoded_is_set<20>(instruction)) ? LDM : STM; 
    }

    if(MATCH(instruction & branch_mask, branch))
    {
       return B;
    }

    if(MATCH(instruction & software_interrupt_mask, software_interrupt))
    {
        return SWI;
    }

    return UNIMP;
}

static constexpr auto ARM_lut = []
{
    constexpr auto LUT_Size = 4096;
    std::array<InstructionMnemonic, LUT_Size> arr = {};
    for (int i = 0; i < LUT_Size; ++i)
    {
        arr[i] = CDecode(i);
    }
    return arr;
}();

void Flush(Core& cpu, bool fetch){
    cpu.Pipeline.decode_stage.mnemonic = NILL;
    cpu.Pipeline.fetch_stage = Fetch(cpu, GetOperatingState(cpu)); //prefetch
    u16 format;
    InstructionMnemonic mnem;
    std::tie(mnem, format) = (GetOperatingState(cpu)) ? Decode_T(cpu.Pipeline.fetch_stage) : std::make_pair(ARM_lut[bit::decode_hash(cpu.Pipeline.fetch_stage)], u16(0));   
    Instruction inst {mnem, cpu.Pipeline.fetch_stage, cpu.rf[PC] -  4, format};
    cpu.Pipeline.decode_stage = inst;
}

int Step(Core& cpu){

    // Log(cpu);

    if(cpu.Pipeline.decode_stage.mnemonic != NILL){
        Execute(cpu, cpu.Pipeline.decode_stage);
    }

    State current_state = GetOperatingState(cpu);
    
    if(cpu.Pipeline.fetch_stage != 0){ 

        u16 format;
        InstructionMnemonic mnem;
        auto test = bit::decode_hash(cpu.Pipeline.fetch_stage);
        std::tie(mnem, format) = (current_state) ? Decode_T(cpu.Pipeline.fetch_stage) : std::make_pair(ARM_lut[test], u16(0));
        Instruction inst {mnem, cpu.Pipeline.fetch_stage, cpu.rf[PC] - ((current_state) ? 2 : 4), format};
        cpu.Pipeline.decode_stage = inst;

    } else cpu.Pipeline.decode_stage.mnemonic = NILL;

    cpu.Pipeline.fetch_stage = Fetch(cpu, current_state);

    //find how the cycles work
    return 0;
}

}; //Namespace ARM