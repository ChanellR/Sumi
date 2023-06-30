#include "arm.hpp"

namespace ARM {

void BranchInstruction(Core& cpu, Instruction inst){
    switch(inst.mnemonic){
        case B: {
            int32_t offset;
            if(GetOperatingState(cpu)){

                switch (inst.format)
                {
                    case 16: {
                        if(!Condition(cpu, inst.attributes.t_Cond)) return;
                        offset = inst.attributes.t_Soffset8 << 1;
                        break;
                    }
                    case 18: {
                        offset = (inst.attributes.t_Offset11 << 1);
                        break;
                    }     
                    case 19: {
                        if(inst.attributes.t_H) { //no BLX yet
                            //low
                            u32 temp = cpu.rf[PC] - 2;
                            cpu.rf[PC] = (cpu.rf[LR] + (inst.attributes.t_Offset << 1)); //pipeline?
                            // if(offset & (0x1 << 22)) {
                            //     offset |= 0xFF800000; //23-bit two's complement offset
                            // }
                            cpu.rf[LR] = temp | 1; //bit zero set
                            // printf("LR: %X\n", cpu.rf[LR]);
                            Flush(cpu);
                            return;
                        } else {
                            cpu.rf[LR] = cpu.rf[PC] + (inst.attributes.t_Offset << 12);
                            return; //don't flush pipeline
                        }
                        break;
                    }       
                }

            } else {
                if(inst.attributes.branch_link) cpu.rf[LR] = cpu.rf[PC] - 4;
                offset = (inst.attributes.branch_offset << 2); 
            }

            cpu.rf[PC] = (u32)(cpu.rf[PC] + offset);
            // printf("PC: %X\n", cpu.rf[PC]);
            Flush(cpu);
            break;
        }
        case BX: {
            //Swap Modes
            PSR cpsr {cpu.rf[CPSR]};
            if(GetOperatingState(cpu)){
                if(inst.attributes.t_H2){
                    //high registers
                    cpu.rf[PC] = cpu.rf[0x8 | inst.attributes.t_RHs] & ~(0x1); //first register bits[3:0]
                    cpsr.operating_state = cpu.rf[0x8 | inst.attributes.t_RHs] & 0x1;
                } else {
                    //low registers
                    cpu.rf[PC] = cpu.rf[inst.attributes.t_RHs] & ~(0x1); //first register bits[3:0]
                    cpsr.operating_state = cpu.rf[inst.attributes.t_RHs] & 0x1;
                     // don't flush on the first instruction of the THUMB
                }
            } else {
                cpu.rf[PC] = (cpu.rf[inst.attributes.branch_Rn] & ~(0x1)); //first register bits[3:0]
                cpsr.operating_state = cpu.rf[inst.attributes.branch_Rn] & 0x1;
                // printf("new pc: %X\n", cpu.rf[PC]);
            }
            
            cpu.rf[CPSR] = cpsr.word;
            Flush(cpu);
            return;
        }
    }
}

}; //Namespace ARM