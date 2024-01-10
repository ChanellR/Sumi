#include "arm.hpp"

namespace ARM {

void MultiplyInstruction(Core& cpu, Instruction inst){
    switch (inst.mnemonic)
    { 
        case MUL: case MLA: {
            u32 mul_result;
            if(GetOperatingState(cpu)){
                mul_result = cpu.rf[inst.attributes.t_Rs] * cpu.rf[inst.attributes.t_Rd];
                cpu.rf[inst.attributes.t_Rd] = mul_result;
            } else {
                mul_result = cpu.rf[inst.attributes.mul_Rm] * cpu.rf[inst.attributes.mul_Rs];
                if(inst.attributes.mull_A) mul_result += cpu.rf[inst.attributes.mul_Rn];
                cpu.rf[inst.attributes.mul_Rd] = mul_result;
            }
            
            if(inst.attributes.mul_S || GetOperatingState(cpu)){
                PSR cpsr {cpu.rf[CPSR]};
                cpsr.V = 0;
                cpsr.C = 0;
                cpsr.N = (mul_result & (0x1 << 31)) ? 1 : 0;
                cpsr.Z = (mul_result == 0);
                cpu.rf[CPSR] = cpsr.word;
            }  
            return;
        }
        case MULL: case MLAL: {
            int64_t mul_result = 0;

            if(inst.attributes.mull_U){ 
                //signed
                mul_result = (int64_t)(int32_t)cpu.rf[inst.attributes.mull_Rm] * (int64_t)(int32_t)cpu.rf[inst.attributes.mull_Rs];
                if(inst.mnemonic == MLAL) {
                    int64_t addend = cpu.rf[inst.attributes.mull_RdHi];
                    addend <<= 32;
                    addend |= cpu.rf[inst.attributes.mull_RdLo];
                    mul_result += addend;
                }
                cpu.rf[inst.attributes.mull_RdHi] = mul_result >> 32;
            } else {
                //unsigned
                mul_result = (uint64_t)cpu.rf[inst.attributes.mull_Rm] * (uint64_t)cpu.rf[inst.attributes.mull_Rs];
                if(inst.mnemonic == MLAL) mul_result +=  ((uint64_t)cpu.rf[inst.attributes.mull_RdHi] << 32) | (uint64_t)cpu.rf[inst.attributes.mull_RdLo];
                cpu.rf[inst.attributes.mull_RdHi] = mul_result >> 32;
            }
            
            cpu.rf[inst.attributes.mull_RdLo] = (u32)mul_result;

            if(inst.attributes.mull_S){
                PSR cpsr {cpu.rf[CPSR]};
                cpsr.N = (mul_result & ((uint64_t)0x1 << 63)) ? 1 : 0;
                cpsr.Z = (mul_result == 0);
                cpu.rf[CPSR] = cpsr.word;
            }   
            return;
        }
    }
}

}; //Namespace ARM