#pragma once

#include "arm.hpp"
#include <stdint.h>

#define HEX( x ) std::setw(2) << std::setfill('0') << std::hex << (int)( x )
#define ROMPAKSIZE 32 * 1024 * 1024
#define SRAMPAKSIZE 64 * 1024
#define CLOCKRATE 0xFFFFFF + 1  //clock-rate 16.78 Mhz 2^24Hz

class GBA {
        
    struct Memory{
        uint8_t SRAM[SRAMPAKSIZE];
        uint8_t ROM[ROMPAKSIZE];
        //different speeds
        uint8_t board_RAM[256 * 1024];
        uint8_t chip_RAM[32 * 1024];

        uint8_t IO[0x3FF];
        uint8_t palette[1024];
        uint8_t VRAM[96 * 1024];
        uint8_t OAM[1024];
    }MEM;
       
    public:
        char filepath[30] {0};

        float FRAME[240 * 160 * 4] {0}; //bitmap (4 bytes per pixel)
        
        void Reset();
        
        void load_rom (uint8_t* dest, size_t file_size);
        void draw_bit_map();
    
        uint32_t memory_access(ARM::MemOp mem_op);
        void stack_dump(char* buffer, uint32_t stack_pointer);
        void mem_dump(char* buffer, uint32_t addr);

};

uint32_t mmu(ARM::MemOp mem_op);