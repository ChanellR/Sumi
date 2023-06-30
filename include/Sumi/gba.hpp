#pragma once

#include "arm.hpp"
#include <stdint.h>

#define HEX( x ) std::setw(2) << std::setfill('0') << std::hex << (int)( x )
#define ROMPAKSIZE 32 * 1024 * 1024
#define SRAMPAKSIZE 64 * 1024
#define CLOCKRATE 0xFFFFFF + 1  //clock-rate 16.78 Mhz 2^24Hz

class GBA {
        
    struct Memory{
        u8 SRAM[SRAMPAKSIZE];
        u8 ROM[ROMPAKSIZE];
        //different speeds
        u8 board_RAM[256 * 1024];
        u8 chip_RAM[32 * 1024];

        u8 IO[0x3FF];
        u8 palette[1024];
        u8 VRAM[96 * 1024];
        u8 OAM[1024];
    }MEM;
       
    public:
        char filepath[30] {0};

        float FRAME[240 * 160 * 4] {0}; //bitmap (4 bytes per pixel)
        
        void Reset();
        
        void load_rom (u8* dest, size_t file_size);
        void draw_bit_map();
    
        u32 memory_access(ARM::MemOp mem_op);
        void stack_dump(char* buffer, u32 stack_pointer);
        void mem_dump(char* buffer, u32 addr);

};

u32 mmu(ARM::MemOp mem_op);