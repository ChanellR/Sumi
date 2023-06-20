#pragma once

#include "arm.hpp"
#include <stdint.h>

#define HEX( x ) std::setw(2) << std::setfill('0') << std::hex << (int)( x )
#define ROMPAKSIZE 32 * 1024 * 1024
#define SRAMPAKSIZE 64 * 1024


class GBA {
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

    public:
        uint8_t ROM[ROMPAKSIZE];
        void load_rom (uint8_t* dest, std::string file_path, size_t file_size);
        uint32_t memory_access(MemOp mem_op);
        void reset();

};

uint32_t mmu(MemOp mem_op);