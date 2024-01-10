#include "arm.hpp"

namespace ARM {

bool Condition(Core& cpu, u8 condition) {
    PSR cpsr {cpu.rf[CPSR]};
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

std::pair<i32, u8> ShiftOperation(Core& cpu, ShiftOp op, u32 value, u16 shifts, bool reg_shift) {
    
    // if(GetOperatingState())printf("op: %d, value: %X, shifts: %d\n", op, value, shifts);
    u32 result = 0;
    uint64_t carry_out = CARRY_VALUE;

    //reg shifting 
    /*
    If the value in the byte is 32 or more, the result will be a logical extension of the shift
    described above:
    1 LSL by 32 has result zero, carry out equal to bit 0 of Rm.
    2 LSL by more than 32 has result zero, carry out zero.
    3 LSR by 32 has result zero, carry out equal to bit 31 of Rm.
    4 LSR by more than 32 has result zero, carry out zero.
    5 ASR by 32 or more has result filled with and carry out equal to bit 31 of Rm.
    6 ROR by 32 has result equal to Rm, carry out equal to bit 31 of Rm.
    7 ROR by n where n is greater than 32 will give the same result and carry out
    as ROR by n-32; therefore repeatedly subtract 32 from n until the amount is
    in the range 1 to 32 and see above.
    Note The zero in bit 7 of an instruction with a register controlled shift is compulsory; a one
    in this bit will cause the instruction to be a multiply or undefined instruction.
    */

   if(reg_shift && shifts == 0) return std::make_pair(value, carry_out);
   if(reg_shift) shifts &= 0xFF;

    switch(op){
        case 0B00:{ //lsl
            if(shifts == 0) return std::make_pair(value, carry_out);
            if(reg_shift){
                if(shifts == 32) return std::make_pair(0, TESTBIT(value, 0));
                if(shifts > 32) return std::make_pair(0, 0);
            }
            if(shifts > 31) {
                result = 0;
                carry_out = (shifts == 32) ? TESTBIT(value, 0) : 0;
                break;
            }
            value <<= (shifts - 1);
            carry_out = (value & 0x80000000) >> 31;
            result = value << 1;
            break;
        }
        case 0B01:{ //lsr
            //lsr #0 is used to encode lsr #32
            //as a result (in the fine print)
            //there is a 0 result and bit 31 is the carry output
            if(shifts == 0 && !reg_shift) return std::make_pair(0, TESTBIT(value, 31));
            if(reg_shift){
                if(shifts == 32) return std::make_pair(0, TESTBIT(value, 31));
                if(shifts > 32) return std::make_pair(0, 0);
            }
            if(shifts > 31) {
                result = 0;
                carry_out = (shifts == 32) ? TESTBIT(value, 31) : 0;
                break;
            }
            value >>= shifts - 1;
            carry_out = (value & 0x1);
            result = value >> 1;
            break;
        }
        case 0B10:{ //asr
            if(shifts == 0) return std::make_pair((TESTBIT(value, 31)) ? 0xFFFFFFFF : 0, TESTBIT(value, 31));
            if(reg_shift){
                if(shifts >= 32) return std::make_pair((TESTBIT(value, 31)) ? 0xFFFFFFFF : 0, TESTBIT(value, 31));
            }
            if (shifts < 32) {
                result = (u32)(((int32_t)value) >> shifts);
                carry_out = (value >> shifts - 1) & 0x1;
            } else {
                carry_out = (value & 0x80000000) ? 1 : 0;
                result = (value & 0x80000000) ? -1 : 0;
            }
            break;
        }
        case 0B11:{ //rr
            if(shifts == 0) {
                uint64_t extended = value;
                extended |= (carry_out << 32);
                extended >>= 1;
                // printf("extended: %X\n", extended);
                return std::make_pair(extended, TESTBIT(value, 0));
            }
            if(reg_shift){
                if(shifts == 32) return std::make_pair(value, TESTBIT(value, 31));
                while(shifts > 32) shifts -= 32;
            }
            for (u8 times = 0; times < shifts; times++){
                carry_out = value & 0x1;
                value >>= 1;
                value |= (carry_out << 31);
            }
            result = value;
            break;
        }
    }

    return std::make_pair(result, carry_out);
}

std::pair<u32, u8> GetOperand(Core& cpu, Instruction inst) {
    //Derives Operand 2 for Data Processing Instructions

    PSR cpsr {cpu.rf[CPSR]};

    u8 carry_out = cpsr.C;
    u32 result = 0;

    if(inst.attributes.dp_I){
        //rotated immediate operand 2 
        u32 imm = (u32)inst.attributes.dp_Imm;
        u8 rotations = inst.attributes.dp_Rotate * 2;
        for (u8 times = 0; times < rotations; times++){
            carry_out = imm & 0x1;
            imm >>= 1;
            imm |= (carry_out << 31);
        }

        result = imm;
        return std::make_pair(result, carry_out);

    } else {

        u32 reg_value = cpu.rf[inst.attributes.dp_Rm];
        if(inst.attributes.dp_Rm == PC && !inst.attributes.dp_I && inst.attributes.dp_reg_shift) reg_value += 4; 
        u16 shifts;
        bool reg_shifting = false;

        if(inst.attributes.dp_reg_shift && !inst.attributes.dp_reg_padding_bit){
            shifts = cpu.rf[inst.attributes.dp_Rs];
            reg_shifting = true;
            // printf("reg shifting: %i\n", reg_shifting);
        } else if(!inst.attributes.dp_reg_shift){
            shifts = inst.attributes.dp_shift_amount;
        } 

        // if(shifts == 0) return std::make_pair(reg_value, carry_out);

        return ShiftOperation(cpu, ShiftOp(inst.attributes.dp_shift_type), reg_value, shifts, reg_shifting);

    }   
}

u32 GetMemAddress(Core& cpu, Instruction inst, bool offset_only) {
    u32 addr_offset = 0;
    /*
        Write-back must not be specified if R15 is specified as the base register (Rn). When
        using R15 as the base register you must remember it contains an address 8 bytes on
        from the address of the current instruction.
        R15 must not be specified as the register offset (Rm).
        When R15 is the source register (Rd) of a register store (STR) instruction, the stored
        value will be address of the instruction plus 12.
    */


    if(inst.attributes.ldst_I){
        inst.attributes.ldst_I = 0;
        addr_offset = GetOperand(cpu, inst).first;
        // printf("offset: %i\n", addr_offset);
        inst.attributes.ldst_I = 1;
    } else {
        addr_offset = inst.attributes.ldst_Imm;
    }

    switch(inst.mnemonic){
        case LDRH: case STRH: case LDRSB: case LDRSH: {
            if(!inst.attributes.ldstx_immediate_type && inst.attributes.ldstx_offset_2 == 0) {
                //register offset
                addr_offset = cpu.rf[inst.attributes.ldstx_Rm];
            } else if (inst.attributes.ldstx_immediate_type) {
                //immediate offset
                addr_offset = (inst.attributes.ldstx_offset_2 << 4) | inst.attributes.ldstx_offset_1;
            }
        }
    }

    // return addr_offset;
    if(offset_only) return addr_offset;

    u32 base = cpu.rf[inst.attributes.ldst_Rn];
    u32 addr = (inst.attributes.ldst_U) ? base + addr_offset : base - addr_offset;

    return addr;
}

}; //Namespace ARM






