#include "arm.hpp"
#include <iostream>

namespace ARM {

void Log(Core& cpu) {
    char file_path[] = "logs/my_thumb_log.bin";
    FILE* f = std::fopen(file_path, "ab");
    if (!f) {
        std::cout << "file not properly initialized" << std::endl;
        return;
    }
    for(int reg = 0; reg < 18; reg++) fwrite(&cpu.rf[reg], sizeof(u32), 1, f);
    std::fclose(f);
}


void RegisterDump (Core& cpu, char* buffer) {
    //make sure to free afterwards!
    char* dump_ptr = buffer;
    for(uint8_t reg = 0; reg < REGSIZE; reg++){
        if(reg < 10){
            dump_ptr += sprintf(dump_ptr, "r%d:   0x%08X\n", reg, cpu.rf[reg]);
        } else if(reg == 16){
            dump_ptr += sprintf(dump_ptr, "cpsr: 0x%08X\n", cpu.rf[reg]);
        } else if(reg == 17){
            dump_ptr += sprintf(dump_ptr, "spsr: 0x%08X\n", cpu.rf[reg]);
        } else {
            dump_ptr += sprintf(dump_ptr, "r%d:  0x%08X\n", reg, cpu.rf[reg]);
        } 
    }
}


void RomDump(Core& cpu, char* buffer){
    u32 pc = cpu.rf[PC];
    for (int word = -15; word < 15; word++){
        if((int32_t)(pc + (word*4)) < 0) continue;

        char mnemonic[15] {0};
        MemOp load = {pc + (word*4), ldw};
        u32 instruction = cpu.MMU(load);

        if(!(pc + (word*4) > 0x8000000 && pc + (word*4) < 0x80000C0)) {
            Disassemble(cpu, mnemonic, instruction, pc + (word*4), GetOperatingState(cpu)); 
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

} //Namespace ARM