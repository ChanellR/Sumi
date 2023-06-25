#include "../include/Sumi/window.hpp"
#include "../include/Sumi/arm.hpp"
#include "../include/Sumi/gba.hpp"

#include <iostream>
#include <iomanip>
#include <string>
#include <cstdio>
#include <cstring>


GBA gba = GBA();
#define DEFAULT_FILE  "roms/1000lsT.gba"
// "gba-tests-master/memory/memory.gba"

uint32_t mmu(ARM::MemOp mem_op){
    return gba.memory_access(mem_op);
}

int main() {

    //probably initialize state before we enter rendering loop. 
    ARMCore core = ARMCore();

    core.SetMMU((void*)mmu); //try to learn how to do this with lambdas or something
    core.Reset();

    sprintf(gba.filepath, DEFAULT_FILE);
    gba.Reset();
    //080008E4
    // uint32_t instruction = 0xE59FD0B8;
    // Instruction inst = {core.Decode(instruction).first, instruction, 0, 0};
    // core.Info(inst);

    
    return run_app(&core, &gba);
}

void GBA::load_rom (uint8_t* dest, size_t file_size){
    //file_size in bytes
    FILE* f = std::fopen(filepath, "rb");
    if (!f) {
        std::cout << "file not properly initialized" << std::endl;
        return;
    }
    if(!std::fread(dest, sizeof(uint8_t), file_size, f)){
        std::cout << "failed to load rom!" << std::endl;
    }
    std::fclose(f);
}

void GBA::Reset(){

    // memset(MEM.SRAM, 0, SRAMPAKSIZE);
    // memset(MEM.board_RAM, 0, 256 * 1024);
    // memset(MEM.chip_RAM, 0, 32 * 1024);
    // memset(MEM.IO, 0, 0x3FF);
    // memset(MEM.palette, 0, 1024);
    // memset(MEM.VRAM, 0, 96 * 1024);
    // memset(MEM.OAM, 0, 1024);
    // memset(ROM, 0, ROMPAKSIZE);

    memset(&MEM, 0, sizeof(MEM));
    gba.load_rom(gba.MEM.ROM, ROMPAKSIZE);
}

void GBA::stack_dump(char* buffer, uint32_t stack_pointer) {
    //showing 4 words below and 10 words above
    for(int word=15; word > -16; word--){
        if(stack_pointer + word < 0) break;
        ARM::MemOp fetch_operation = {stack_pointer + (word*4), ARM::ldw};
        buffer += sprintf(buffer, "0x%08X %08Xh", stack_pointer + (word*4), memory_access(fetch_operation));
        if(word == 0){
            buffer += sprintf(buffer, "<-\n");
        } else {
            buffer += sprintf(buffer, "\n");
        }
    } 
}

void GBA::mem_dump(char* buffer, const uint32_t addr){
    ARM::MemOp fetch_operation;
    for(int word = 0; word < 20; word++){
    fetch_operation = {addr + (word*4), ARM::ldw};
        buffer += sprintf(buffer, "0x%08X %08Xh", addr + (word * 4), memory_access(fetch_operation));
    }
}

uint32_t GBA::memory_access(ARM::MemOp mem_op){

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
        if(addr == DISPSTAT) {
            *location ^= 0x1;
            return *location; 
        }
    }
    else if(addr >= 0x04000400 && addr <= 0x04FFFFFF){
        //NU
    }
    else if(addr >= 0x05000000 && addr <= 0x050003FF){
        //Palette
        location = &(MEM.palette[addr-0x05000000]);
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
        location = &MEM.ROM[addr-0x08000000];
    }
    else if(addr >= 0x0A000000 && addr <= 0x0BFFFFFF){
        //ROM Wait 1
        location = &MEM.ROM[addr-0x0A000000];
    }
    else if(addr >= 0x0C000000 && addr <= 0x0DFFFFFF){
        //ROM Wait 2
        location = &MEM.ROM[addr-0x0C000000];
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

    if (location == nullptr) {
        return 0;
    }
    uint32_t load_val;

    //Little-endian
    switch(mem_op.operation){
        case ARM::ldb: {
            load_val = *location;
            break;
        }
        case ARM::ldh: {
            load_val = *location;
            load_val |= *(location + 1) << 8;
            break;
        }
        case ARM::ldsb: {
            load_val = (uint32_t)(int8_t)*location;
            break;
        }
        case ARM::ldsh: {
            load_val = *location;
            load_val |= (uint32_t)((int8_t)*(location + 1)) << 8;
            break;
        }
        case ARM::ldw: {
            load_val = *location;
            load_val |= *(location + 1) << 8;
            load_val |= *(location + 2) << 16;
            load_val |= *(location + 3) << 24;
            break;
        }
        case ARM::strb: {
            *location = (uint8_t)mem_op.data;
            break;
        }
        case ARM::strh: {
            *location = (uint8_t)(mem_op.data & 0xff);
            *(location + 1) = (uint8_t)((mem_op.data & 0xff00) >> 8);
            break;
        }
        case ARM::strw: {
            *location = (uint8_t)(mem_op.data & 0xff);
            *(location + 1) = (uint8_t)((mem_op.data & 0xff00) >> 8);
            *(location + 2) = (uint8_t)((mem_op.data & 0xff0000) >> 16);
            *(location + 3) = (uint8_t)((mem_op.data & 0xff000000) >> 24);
            break;
        }
    }

    return load_val;
}

void GBA::draw_bit_map(){

    /*
        BG Mode 4 - 240x160 pixels, 256 colors (out of 32768 colors)
        One byte is associated to each pixel, selecting one of the 256 palette entries.
        Color 0 (backdrop) is transparent, and OBJs may be displayed behind the bitmap.
        The first 240 bytes define the topmost line, the next 240 the next line, and so on.
        The background occupies 37.5 KBytes,
        allowing two frames to be used (06000000-060095FF for Frame 0, and 0600A000-060135FF for Frame 1).

        Color Definitions
        Each color occupies two bytes (same as for 32768 color BG modes):
        Bit   Expl.
        0-4   Red Intensity   (0-31)
        5-9   Green Intensity (0-31)
        10-14 Blue Intensity  (0-31)
        15    Not used
    */
    
    union Color {
        uint16_t palette_val;
        struct {
            unsigned int Red : 5; //(0-31)
            unsigned int Green : 5;
            unsigned int Blue : 5;
            int : 1;
        };
    };

    Color color {0};
    //The BG palette is just and array of 2byte colors that is 256 elements long
    for(uint32_t pixel = 0; pixel < 240 * 160; pixel++){
  
        uint8_t palette_num = MEM.VRAM[pixel];
        color.palette_val = ((uint16_t)(MEM.palette[palette_num * 2 + 1]) << 8) | (MEM.palette[palette_num * 2]);

        FRAME[(pixel * 4)] = (color.Red / 31.0f);
        FRAME[(pixel * 4) + 1] = (color.Green / 31.0f);
        FRAME[(pixel * 4) + 2] = (color.Blue / 31.0f);
        FRAME[(pixel * 4) + 3] = 1.0f; //alpha

    }
}