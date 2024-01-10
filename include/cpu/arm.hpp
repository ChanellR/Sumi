#pragma once
#include <stdint.h>
#include <string>
#include <cstring>
#include <array>
#include <Instructions.hpp>
#include <Memory.hpp>

#define NOP -1
#define REGSIZE 18

#define SP 13
#define LR 14
#define PC 15
#define CPSR 16
#define SPSR 17

#define BIT_N 31
#define BIT_Z 30 
#define BIT_C 29 
#define BIT_V 28 

#define N_FIELD 0x1 << 31
#define Z_FIELD 0x1 << 30
#define C_FIELD 0x1 << 29
#define V_FIELD 0x1 << 28

#define CARRY_VALUE (cpu.rf[CPSR] & 0x20000000) >> 29
#define BIT_L 24
#define BIT_S 20
#define BIT_loadStore 20

#define CONDITION( x ) (u8)((x & 0xf0000000) >> 28)
//match the nibble starting at the bit given
#define MATCHNIBBLE(x, template, bit) !((x & (0xf << bit)) ^ (template << bit))
#define MATCH(x, template)  !((x) ^ template)
#define GETNIBBLE(x, bit) (u8)((x & (0xf << bit)) >> bit)
#define TESTBIT(x, bit) (x & (0x1 << bit)) >> bit

#define KEYINPUT 0x4000130
#define DISPSTAT 0x4000004

#define STEPSPERSEC 0x9F

using u32 = uint32_t;
using u16 = uint16_t;
using u8 = uint8_t;
using i32 = int32_t;

class Memory;

namespace ARM {
    
    enum State {ARMMODE, THUMBMODE};
    
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
    
    // enum OpType {
    //     strw,
    //     strb,
    //     ldw, 
    //     ldb, 
    //     ldh,
    //     ldsh,
    //     ldsb, 
    //     strh,
    // };

    // struct MemOp {
    //     u32 addr;
    //     OpType operation;
    //     u32 data; //on store
    // };
    
    //how they are written in ARM assembly
    enum DisplayType {
        dp_1, // ? Rd,<Op2>
        dp_2, // ? Rn,<Op2>
        dp_3, // ? Rd,Rn,<Op2>
        b_1,  // B{L} <expression>
        b_2,  // BX Rn
        ldst,
        ldstM,
        swp, //Rd,Rm,[Rn]
        mul,
        NOOP, // ?
    }; 
    
    
    enum ShiftOp {
        lsl, lsr, asr, rr
    };

    //Program Status Registers
    union PSR  {
        u32 word;
        struct {
            unsigned int operating_mode : 5;
            unsigned int operating_state : 1; //Thumb/Arm
            unsigned int interrupt_disable : 2;
            int : 20;
            unsigned int V : 1; //overflow
            unsigned int C : 1; //Carry / Borrow / Extend
            unsigned int Z : 1; //Zero
            unsigned int N : 1; //Negative / Less Than
        };
        struct {
            int : 28;
            unsigned int flags : 4;
        };
    };

    struct RegisterFile {        

        u32 system_registers[17];
        u32 fiq_registers[8];
        u32 svc_registers[3];
        u32 abt_registers[3];
        u32 irq_registers[3];
        u32 und_registers[3];

        public: 
        
            void Reset() {
                memset(system_registers, 0, 16 * 4);
                memset(fiq_registers, 0, 8 * 4);
                memset(svc_registers, 0, 3 * 4);
                memset(abt_registers, 0, 3 * 4);
                memset(irq_registers, 0, 3 * 4);
                memset(und_registers, 0, 3 * 4);
                system_registers[CPSR] = 0x000000DF; //ARM, System mode, int enabled
                system_registers[SP] = 0x03007F00; //Stack starting position
                system_registers[PC] = 0x80000000; //start at ROM execution
            }

            u32& operator[](u16 reg_idx) {
                switch(reg_idx){
                    case PC: return system_registers[PC];
                    case CPSR: return system_registers[CPSR];
                }
                switch(PSR{system_registers[CPSR]}.operating_mode){
                    case 0B10000: case 0B11111:{ //User & System Mode
                        if(reg_idx == SPSR) return system_registers[CPSR]; //SPSR and CPSR are same
                        return system_registers[reg_idx];
                    }
                    case 0B10001: { //FIQ Mode
                        if(reg_idx < 8) return system_registers[reg_idx];
                        if(reg_idx == SPSR) return fiq_registers[7]; //special banked SPSR 
                        return fiq_registers[reg_idx - 8];
                    }
                    case 0B10010: { //IRQ Mode
                        if(reg_idx < 13) return system_registers[reg_idx];
                        if(reg_idx == SPSR) return irq_registers[2]; //special banked SPSR 
                        return irq_registers[reg_idx - 13];
                    }
                    case 0B10011: { //Supervisor Mode
                        if(reg_idx < 13) return system_registers[reg_idx];
                        if(reg_idx == SPSR) return svc_registers[2]; //special banked SPSR 
                        return svc_registers[reg_idx - 13];
                    }
                    case 0B10111: { //Abort Mode
                        if(reg_idx < 13) return system_registers[reg_idx];
                        if(reg_idx == SPSR) return abt_registers[2]; //special banked SPSR 
                        return abt_registers[reg_idx - 13];
                    }
                    case 0B11011: { //Undefined Mode
                        if(reg_idx < 13) return system_registers[reg_idx];
                        if(reg_idx == SPSR) return und_registers[2]; //special banked SPSR 
                        return und_registers[reg_idx - 13];
                    }
                }
                return system_registers[0]; //shouldn't happen
            }

            const u32& operator[](u16 reg_idx) const {
            switch(reg_idx){
                case PC: return system_registers[PC];
                case CPSR: return system_registers[CPSR];
            }
            switch(PSR{system_registers[CPSR]}.operating_mode){
                case 0B10000: case 0B11111:{ //User & System Mode
                    if(reg_idx == SPSR) return system_registers[CPSR]; //SPSR and CPSR are same
                    return system_registers[reg_idx];
                }
                case 0B10001: { //FIQ Mode
                    if(reg_idx < 8) return system_registers[reg_idx];
                    if(reg_idx == SPSR) return fiq_registers[7]; //special banked SPSR 
                    return fiq_registers[reg_idx - 8];
                }
                case 0B10010: { //IRQ Mode
                    if(reg_idx < 13) return system_registers[reg_idx];
                    if(reg_idx == SPSR) return irq_registers[2]; //special banked SPSR 
                    return irq_registers[reg_idx - 13];
                }
                case 0B10011: { //Supervisor Mode
                    if(reg_idx < 13) return system_registers[reg_idx];
                    if(reg_idx == SPSR) return svc_registers[2]; //special banked SPSR 
                    return svc_registers[reg_idx - 13];
                }
                case 0B10111: { //Abort Mode
                    if(reg_idx < 13) return system_registers[reg_idx];
                    if(reg_idx == SPSR) return abt_registers[2]; //special banked SPSR 
                    return abt_registers[reg_idx - 13];
                }
                case 0B11011: { //Undefined Mode
                    if(reg_idx < 13) return system_registers[reg_idx];
                    if(reg_idx == SPSR) return und_registers[2]; //special banked SPSR 
                    return und_registers[reg_idx - 13];
                }
            }
            return system_registers[0]; //shouldn't happen
        }

    };

    struct ControlSignals {
        u8 BIGEND : 1;
    };

    //Pipeline
    struct CpuPipeline {
        u32 fetch_stage;
        Instruction decode_stage;
    };

    struct Core {

        RegisterFile rf;
        CpuPipeline Pipeline;
        Memory& mem;
        // u32 (*MMU)(MemOp mem_op);

        Core(Memory& memory) : mem(memory) {}
    };
    
    std::pair<InstructionMnemonic, u16> Decode(u32 instruction);
    constexpr std::pair<InstructionMnemonic, u16> Decode_T(u16 instruction);
    constexpr InstructionMnemonic CDecode(const u32 instruction);

    bool Condition(Core& cpu, u8 condition);
    std::pair<i32, u8> ShiftOperation(Core& cpu, ShiftOp op, u32 value, u16 shifts, bool reg_shift=false);
    std::pair<u32, u8> GetOperand(Core& cpu, Instruction inst);
    u32 GetMemAddress(Core& cpu, Instruction inst, bool offset_only=false);

    void BranchInstruction(Core& cpu, Instruction inst);
    void DataProcessingInstruction(Core& cpu, Instruction inst);
    void DataTransferInstruction(Core& cpu, Instruction inst);
    void MultiplyInstruction(Core& cpu, Instruction inst);

    void Flush(Core& cpu, bool fetch=true);
    u32 Fetch(Core& cpu, State current_state);
    void Execute(Core& cpu, Instruction inst);

    //Essentials
    // void SetMMU(Core& cpu, void* func_ptr);
    void Reset(Core& cpu);
    int Step(Core& cpu);

    u32 GetReg(Core& cpu, u16 reg);
    void SetReg(Core& cpu, u16 reg, u32 value);
    State GetOperatingState(Core& cpu);
    u32 GetExecuteStageAddr(Core& cpu);

    void RegisterDump(Core& cpu, char* buffer);
    void RomDump(Core& cpu, char* buffer);

    void Disassemble(Core& cpu, char* buffer, u32 instruction, u32 instruction_addr, bool Thumb);
    void Log(Core& cpu);

    ARM::DisplayType GetDisplayType(InstructionMnemonic mnemonic);
    void Info(Core& cpu, Instruction instruction);

}; //Namespace ARM


