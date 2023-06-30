#pragma once
#include "arm.hpp"
#include "gba.hpp"

#include <set>
#include <map>

struct EmulatorSettings {
    bool enable_debug;

    bool enable_reg_file;
    bool enable_stack;
    bool enable_instructions;
    bool enable_break_points;
    bool enable_memory;
    bool enable_controls;
    
    bool running;
    int  search_address; 
    bool enable_video; 
};

struct EmulatorData {
    char reg_dump_buffer[20 * REGSIZE];
    char stack_dump_buffer[24 * 35];
    char breakpoints_buffer[8 * 10]; //8 points
    std::set<u32> breakpoints;
    std::map<u16, uint8_t> key_presses;
};

int run_app(ARM::Core& arm_handle, GBA* gba_handle);
