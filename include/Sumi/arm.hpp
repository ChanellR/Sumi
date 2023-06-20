#pragma once
#include <stdint.h>
#include <string>

#define SP 13
#define LR 14
#define PC 15
#define CPSR 16

#define N 0x80000000
#define Z 0x40000000
#define C 0x20000000
#define V 0x10000000

class Arm {

    enum OpType {ldb, ldw, ldword, strb, strw};

    struct MemOp {
        uint32_t addr;
        OpType operation;
        uint32_t data; //on store
    };

    enum State {arm_mode, thumb_mode};

    //Program Status Registers
    struct PSR {
        uint8_t operating_mode : 5;
        State operating_state : 1; //Thumb/Arm
        uint8_t interrupt_disable : 2;
        uint32_t : 20;
        uint8_t condition_flags : 4;
    };

    enum InstructionMnemonic {
        B,
        UNIMP,
    };

    enum ConditionField {
        EQ, //equal
        NE, //not equal
        CS, //unsigned higher or same
        CC, //unsigned lower
        MI, //negative
        PL, //positive or zero
        VS, //overflow
        VC, //no overflow
        HI, //unsigned higher
        LS, //unsigned lower or same
        GE, //greater or equal
        LT, //less than
        GT, //greater than
        LE, //less than or equal
        AL, //always
    };


    uint32_t rf[17]; //register file
    //I don't need to store memory here I believe, I need only a memory access handler
    uint32_t (*MMU)(MemOp mem_op);
    size_t memory_size;

    //todo:
    //interrupts
    //cycle timing

    uint32_t fetch();
    uint8_t decode(uint32_t instruction);
    uint8_t execute(uint32_t instruction, InstructionMnemonic type);
    
    public:
        void reset();
        void step();
        char* dump();
        void load_rom (uint8_t* dest, std::string file_path, size_t file_size);

};
    

