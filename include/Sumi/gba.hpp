#pragma once

#include "arm.hpp"
#include <stdint.h>

#define HEX( x ) std::setw(2) << std::setfill('0') << std::hex << (int)( x )
#define ROMPAKSIZE 32 * 1024 * 1024
#define SRAMPAKSIZE 64 * 1024
#define CLOCKRATE 0xFFFFFF + 1  //clock-rate 16.78 Mhz 2^24Hz

class GBA {
       
    public:
        //memory structuring 
        struct Memory{
            uint8_t SRAM[SRAMPAKSIZE];
            //different speeds
            uint8_t board_RAM[256 * 1024];
            uint8_t chip_RAM[32 * 1024];
            uint8_t IO[0x3FF];
            uint8_t palette[1024];
            uint8_t VRAM[96 * 1024];
            uint8_t OAM[1024];
        }MEM;

        uint8_t ROM[ROMPAKSIZE];
        float FRAME[240 * 160 * 4]; //bitmap (4 bytes per pixel)
        void load_rom (uint8_t* dest, size_t file_size);
        uint32_t memory_access(Arm::MemOp mem_op);
        void draw_bit_map();
        void reset();
        void stack_dump(char* buffer, uint32_t stack_pointer);
        void mem_dump(char* buffer, uint32_t addr);
        char filepath[30] {0};

};

uint32_t mmu(Arm::MemOp mem_op);