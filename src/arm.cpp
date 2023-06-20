#include "../include/Sumi/arm.hpp"
#include <cstdio>
#include <iostream>
#include <string>
#include <stdint.h>

#include <cstring>

void Arm::reset(){
    //sets all registers to 0
    memset(rf, 0, 4*17);
    //for now
    rf[PC] = 0x08000000;
}

uint32_t Arm::fetch(){
    //just fetching double-words for now
    MemOp fetch_operation = {rf[PC], ldd};
    uint32_t instruction = MMU(fetch_operation);
    rf[PC] += 4;
    return instruction;
}

bool Arm::condition_pass(ConditionField condition){
    switch (condition)
    {
        case AL: 
            return true;
            break;
    }
    return false;
}

uint8_t Arm::decode(uint32_t instruction){
    //what paradigms are there to decode
    if(!condition_pass((ConditionField)CONDITION(instruction))) {
        return (uint8_t)UNIMP;
    }

    if (MATCHNIBBLE(instruction, 0xA, 6)) {
        return (uint8_t)B;
    }
    
    return (uint8_t)UNIMP;
}

void encode(char* buffer, uint32_t instruction){
    //for translating into readable syntax
}


void Arm::execute(uint32_t instruction, InstructionMnemonic type){
    switch(type){
        case B: {
            //link bit
            if (instruction & 0x01000000) rf[LR] = rf[PC]; 
            //offset shifting and sign-extending
            int32_t offset = (int32_t)(instruction & 0x00800000) ? (int32_t)(0xfc000000) : 0;
            offset |= (instruction & 0x00ffffff) << 2;
            //pc branching
            //why am I 4 off? I guess
            rf[PC] = (uint32_t)(rf[PC] + offset + 4);
            break;
        }
        case UNIMP: {
            break;
        }
    }
    //is return value needed?
}

void Arm::step(){
    uint32_t instruction = fetch();
    InstructionMnemonic instruction_type = (InstructionMnemonic)decode(instruction);
    printf("Fetched instruction: 0x%08X type:%d\n", instruction, instruction_type);
    execute(instruction, instruction_type);
}

void Arm::dump (char* buffer) const{
    //make sure to free afterwards!
    char* dump_ptr = buffer;
    for(uint8_t reg = 0; reg < REGSIZE; reg++){
        if(reg < 10){
            dump_ptr += sprintf(dump_ptr, "r%d:  0x%08X\n", reg, rf[reg]);
        } else {
            dump_ptr += sprintf(dump_ptr, "r%d: 0x%08X\n", reg, rf[reg]);
        } 
    }
}

void Arm::set_MMU(void* func_ptr){
    MMU = (uint32_t(*)(MemOp)) func_ptr;
}

uint32_t Arm::get_pc() const{
    return rf[PC];
}