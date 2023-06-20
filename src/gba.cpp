#include "../include/Sumi/window.hpp"
#include "../include/Sumi/arm.hpp"
#include "../include/Sumi/gba.hpp"

#include <iostream>
#include <iomanip>
#include <string>
#include <cstdio>
#include <cstring>

GBA gba = GBA();

int main() {

    //probably initialize state before we enter rendering loop. 
    Arm arm = Arm();

    arm.set_MMU((void*)mmu);
    arm.reset();
    gba.reset();

    return run_app(&arm, &gba);
}

uint32_t mmu(MemOp mem_op){
    return gba.memory_access(mem_op);
}

void GBA::load_rom (uint8_t* dest, std::string file_path, size_t file_size){
    //file_size in bytes
    FILE* f = std::fopen(file_path.c_str(), "rb");
    if (!f) {
        std::cout << "file not properly initialized" << std::endl;
        return;
    }
    if(!std::fread(dest, sizeof(uint8_t), file_size, f)){
        std::cout << "failed to load rom!" << std::endl;
    }
    std::fclose(f);
}

void GBA::reset(){
    memset(MEM.SRAM, 0, SRAMPAKSIZE);
    memset(MEM.board_RAM, 0, 256 * 1024);
    memset(MEM.chip_RAM, 0, 32 * 1024);
    memset(MEM.IO, 0, 0x3FF);
    memset(MEM.palette, 0, 1024);
    memset(MEM.VRAM, 0, 96 * 1024);
    memset(MEM.OAM, 0, 1024);
    memset(ROM, 0, ROMPAKSIZE);
    gba.load_rom(gba.ROM, std::string("roms/CPUTest.gba"), ROMPAKSIZE);
}

uint32_t GBA::memory_access(MemOp mem_op){

    // printf("memory.access: addr: %08X mem_op: %d\n", mem_op.addr, mem_op.operation);
    uint32_t addr = mem_op.addr;
    uint8_t* location = nullptr;
    
    if(addr >= 0x00000000 && addr <= 0x00003FFF){
        //BIOS
        //some type of handling
    }
    else if(addr >= 0x00004000 && addr <= 0x01FFFFFF){
        //not used
    }
    else if(addr >= 0x02000000 && addr <= 0x0203FFFF){
        //board_RAM
        location = &MEM.board_RAM[addr-0x02000000];
    }
    else if(addr >= 0x02040000 && addr <= 0x02FFFFFF){
        //NU
    }
    else if(addr >= 0x03000000 && addr <= 0x03007FFF){
        //chip_RAM
        location = &MEM.chip_RAM[addr-0x03000000];
    }
    else if(addr >= 0x03008000 && addr <= 0x03FFFFFF){
        //NU
    }
    else if(addr >= 0x04000000 && addr <= 0x040003FE){
        //IO
        location = &MEM.IO[addr-0x04000000];
    }
    else if(addr >= 0x04000400 && addr <= 0x04FFFFFF){
        //NU
    }
    else if(addr >= 0x05000000 && addr <= 0x050003FF){
        //Palette
        location = &MEM.palette[addr-0x05000000];
    }
    else if(addr >= 0x05000400 && addr <= 0x05FFFFFF){
        //NU
    }
    else if(addr >= 0x06000000 && addr <= 0x06017FFF){
        //VRAM
        location = &MEM.VRAM[addr-0x06000000];
    }
    else if(addr >= 0x06018000 && addr <= 0x06FFFFFF){
        //NU
    }
    else if(addr >= 0x07000000 && addr <= 0x070003FF){
        //OAM
        location = &MEM.OAM[addr-0x07000000];
    }
    else if(addr >= 0x07000400 && addr <= 0x07FFFFFF){
        //NU
    }
    else if(addr >= 0x08000000 && addr <= 0x09FFFFFF){
        //ROM Wait 0
        location = &ROM[addr-0x08000000];
    }
    else if(addr >= 0x0A000000 && addr <= 0x0BFFFFFF){
        //ROM Wait 1
        location = &ROM[addr-0x0A000000];
    }
    else if(addr >= 0x0C000000 && addr <= 0x0DFFFFFF){
        //ROM Wait 2
        location = &ROM[addr-0x0C000000];
    }
    else if(addr >= 0x0E000000 && addr <= 0x0E00FFFF){
        //SRAM 
        location = &MEM.SRAM[addr-0x0E000000];
    }
    else if(addr >= 0x0E010000 && addr <= 0x0FFFFFFF){
        //NU
    }
    else if (addr >= 0x10000000 && addr <= (uint32_t)-1){
        //NU
    }

    if (location == nullptr) return 0;
    
    uint32_t load_val;

    //Little-endian
    switch(mem_op.operation){
        case ldb: {
            load_val = *location;
            break;
        }
        case ldw: {
            load_val = *location;
            load_val |= *(location + 1) << 8;
            break;
        }
        case ldd: {
            load_val = *location;
            load_val |= *(location + 1) << 8;
            load_val |= *(location + 2) << 16;
            load_val |= *(location + 3) << 24;
            break;
        }
        case strb: {
            *location = (uint8_t)mem_op.data;
            break;
        }
        case strw: {
            *location = (uint8_t)(mem_op.data & 0xff);
            *(location + 1) = (uint8_t)((mem_op.data & 0xff00) >> 8);
            break;
        }
        case strd: {
            *location = (uint8_t)(mem_op.data & 0xff);
            *(location + 1) = (uint8_t)((mem_op.data & 0xff00) >> 8);
            *(location + 2) = (uint8_t)((mem_op.data & 0xff0000) >> 16);
            *(location + 3) = (uint8_t)((mem_op.data & 0xff000000) >> 24);
            break;
        }
    }

    return load_val;
}