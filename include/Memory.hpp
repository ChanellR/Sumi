#ifndef MEMORY_H
#define MEMORY_H

#include <arm.hpp>
#include <string>
#include <iostream>

#define HEX( x ) std::setw(2) << std::setfill('0') << std::hex << (int)( x )
#define ROMPAKSIZE 32 * 1024 * 1024
#define SRAMPAKSIZE 64 * 1024
#define CLOCKRATE 0xFFFFFF + 1  //clock-rate 16.78 Mhz 2^24Hz

enum OpType {
    strw,
    strb,
    ldw, 
    ldb, 
    ldh,
    ldsh,
    ldsb, 
    strh,
};

struct MemOp {
    u32 addr;
    OpType operation;
    u32 data; //on store
};

struct GBA_Memory {
    u8 SRAM[SRAMPAKSIZE];
    u8 ROM[ROMPAKSIZE];
    //different speeds
    u8 board_RAM[256 * 1024];
    u8 chip_RAM[32 * 1024];

    u8 IO[0x3FF];
    u8 palette[1024];
    u8 VRAM[96 * 1024];
    u8 OAM[1024];
};



class Memory
{
public:
    u32 access(MemOp mem_op);

    void Reset()
    {
        memset(&bus, 0, sizeof(GBA_Memory));
        if (!current_rom_path.empty())
            LoadRom(current_rom_path);
    }

    void LoadRom(std::string file_path)
    {
        current_rom_path = file_path;
        //file_size in bytes
        FILE* f = std::fopen(file_path.c_str(), "rb");
        if (!f) {
            std::cout << "file not properly initialized" << std::endl;
            return;
        }
        if(!std::fread(bus.ROM, sizeof(uint8_t), ROMPAKSIZE, f)){
            std::cout << "failed to load rom!" << std::endl;
        }
        std::fclose(f);
    }

    GBA_Memory bus;

private:
    std::string current_rom_path;
};

#endif
