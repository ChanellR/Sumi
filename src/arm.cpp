#include "../include/Sumi/arm.hpp"
#include <cstdio>
#include <iostream>
#include <string>
#include <stdint.h>

#define REGSIZE 17

void Arm::load_rom (uint8_t* dest, std::string file_path, size_t file_size){
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

void Arm::reset(){

    //reset memory
    MemOp clear = {0, OpType::strb, (uint32_t)0};
    for(uint32_t byte = 0; byte < memory_size; byte++){
        clear.addr = byte;
        MMU(clear);
    }

    //sets all registers to 0
    memset(rf, 0, 4*17);
}

uint32_t Arm::fetch(){
    //just fetching double-words for now
    MemOp fetch_operation = {rf[PC], ldword};
    uint32_t instruction = MMU(fetch_operation);
    rf[PC] += 4;
    return instruction;
}

uint8_t Arm::decode(uint32_t instruction){

}

uint8_t Arm::execute(uint32_t instruction, InstructionMnemonic type){

}

void Arm::step(){
    uint32_t instruction = fetch();
    execute(instruction, (InstructionMnemonic)decode(instruction));
}

char* Arm::dump(){
    for(uint8_t reg = 0; reg < REGSIZE; reg++){
        
    }
}