#pragma once
#include <stdint.h>
#include <string>
#include <cstring>
#include <array>

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

namespace ARM {
    
    enum State {ARMMODE, THUMBMODE};
    
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

    struct ControlSignals {
        u8 BIGEND : 1;
    };

    enum InstructionMnemonic {
        AND, EOR, SUB, RSB, ADD, ADC, SBC, RSC,
        TST, TEQ, CMP, CMN, ORR, MOV, BIC, MVN,
        B, UNDEF, UNIMP, MRS, MSR, BX, LDR, STR,
        SWI, LDRH, STRH, LDRSB, LDRSH, LDM, STM,
        MSRf, SWP, MUL, MLA, MULL, MLAL, NILL
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
    
    union InstructionAttributes { //instruction interface

        //could also sort this by tab
        u32 word;

        struct { //condition
            int : 28;
            unsigned int Cond : 4;
        };

        struct { //branch
            unsigned int branch_Rn : 4;
        };

        struct { //branch
            int branch_offset : 24;
            unsigned int branch_link : 1;
            int : 3;
        };

        struct {
            int : 4;
            unsigned int status_msr_valid : 8;
            unsigned int status_valid1 : 4;
            unsigned int status_mask : 4;
            unsigned int : 1;
            unsigned int status_toPsr : 1;
            unsigned int : 1;
            unsigned int status_valid2 : 2;
        };

        struct { //data processing
            unsigned int dp_operand_2 : 12;
            unsigned int dp_Rd : 4;
            unsigned int dp_Rn : 4;
            unsigned int dp_S : 1;
            unsigned int dp_OpCode : 4;
            unsigned int dp_I : 1;
            unsigned int dp_valid : 2;
        };

        struct { //data processing
            unsigned int dp_Rm : 4;
            unsigned int dp_Shift : 4;
            int : 14;
            unsigned int msr_P : 1;
        };

        struct { //data processing
            int : 4;
            unsigned int dp_reg_shift : 1;
            unsigned int dp_shift_type : 2;
            unsigned int dp_reg_padding_bit : 1;
            unsigned int dp_Rs : 4;
        };

        struct { //data processing
            int : 4;
            int : 3;
            unsigned int dp_shift_amount : 5;
        };

        struct { //data processing
            unsigned int dp_Imm : 8;
            unsigned int dp_Rotate : 4;
        };

        struct { //data transfer
            unsigned int ldst_Imm : 12; //Offset
            unsigned int ldst_Rd : 4;
            unsigned int ldst_Rn : 4;
            unsigned int ldst_L : 1; //Load/Store bit
            unsigned int ldst_W : 1; //Write-back bit
            unsigned int ldst_B : 1; //Byte/Word bit
            unsigned int ldst_U : 1; //Up/Down bit
            unsigned int ldst_P : 1; //Pre/Post Indexing bit
            unsigned int ldst_I : 1; // Immediate Offset Toggle
            unsigned int ldst_identifier : 2; //valid:　０１ 
        };

        struct { //data transfer
            unsigned int ldst_Rm : 4;
            unsigned int ldst_Shift : 8;
        };

        struct { //half/signed data transfer
            int ldstx_Rm: 4; //for load_store extended
            unsigned int ldstx_valid_1 : 1;
            unsigned int ldstx_SH : 2;
            unsigned int ldstx_valid_2 : 1;
            unsigned int ldstx_offset_2 : 4;
            unsigned int ldstx_Rd : 4;
            unsigned int ldstx_Rn : 4;
            unsigned int ldstx_L : 1; //Load/Store bit
            unsigned int ldstx_W : 1; //Write-back bit
            unsigned int ldstx_immediate_type : 1; //immediate offset/register offset
            unsigned int ldstx_U : 1; //Up/Down bit
            unsigned int ldstx_P : 1; //Pre/Post Indexing bit
            unsigned int ldstx_valid_3 : 3;
        };

        struct { //half/signed data transfer
            unsigned int ldstx_offset_1 : 4;
        };

        struct { //block data transfer
            unsigned int ldstM_register_list : 16;
            unsigned int ldstM_Rn : 4;
            unsigned int ldstM_L : 1;
            unsigned int ldstM_W : 1;
            unsigned int ldstM_S : 1;
            unsigned int ldstM_U : 1;
            unsigned int ldstM_P : 1;
            unsigned int ldstM_valid : 3; //0B100 for STM/LDM
        };

        struct { //multiply/Accumulate
            unsigned int mul_Rm : 4;
            unsigned int mul_identifier_1 : 4;
            unsigned int mul_Rs : 4;
            unsigned int mul_Rn : 4;
            unsigned int mul_Rd : 4;
            unsigned int mul_S : 1;
            unsigned int mul_A : 1;
            unsigned int mul_identifier_2 : 6;
        };

        struct { //multiply long
            unsigned int mull_Rm : 4;
            unsigned int mull_identifier_1 : 4;
            unsigned int mull_Rs : 4;
            unsigned int mull_RdLo : 4;
            unsigned int mull_RdHi : 4;
            unsigned int mull_S : 1;
            unsigned int mull_A : 1;
            unsigned int mull_U : 1;
            unsigned int mull_identifier_2 : 5;
        };

        //THUMB INSTRUCTIONS
        struct {
            unsigned int t_Rd : 3;
            unsigned int t_Rs : 3;
            unsigned int t_Offset5 : 5;
            unsigned int t_f1_Opcode : 2;
            unsigned int t_f1_indentifier : 3; //format 1 identitifier
        };

        struct {
            unsigned int : 6;
            unsigned int t_Rn_Offset3 : 3;
            unsigned int t_f2_Opcode : 1;
            unsigned int t_I : 1;
        };

        struct {
            unsigned int t_Offset8 : 8;
            unsigned int t_f3_Rd : 3;
            unsigned int t_f3_Opcode : 2;
            unsigned int t_f3_indentifier : 3; //format 3 identitifier
        };

        struct {
            unsigned int : 6;
            unsigned int t_f4_Opcode : 4;
            unsigned int t_f4_identifier : 6;
        };

        struct {
            unsigned int t_RHd : 3;
            unsigned int t_RHs : 3;
            unsigned int t_H2 : 1;
            unsigned int t_H1 : 1;
            unsigned int t_f5_Opcode : 2;
            unsigned int t_f5_identifier : 6;
        };

        struct {
            unsigned int t_Word8 : 8;
            unsigned int t_f6_Rd : 3;
            unsigned int t_f6_identifier : 5;
        };

        struct {
            unsigned int : 3;
            unsigned int t_Rb : 3;
            unsigned int t_Ro : 3;
            unsigned int t_f7_identifier1 : 1;
            unsigned int t_f7_B : 1;
            unsigned int t_L : 1;
            unsigned int t_f7_identifier2 : 4;
        };

        struct {
            unsigned int : 9;
            unsigned int t_f8_identifier1 : 1;
            unsigned int t_f8_S : 1;
            unsigned int t_H : 1;
            unsigned int t_f8_identifier2 : 4;
        };

        struct {
            unsigned int : 11;
            unsigned int : 1;
            unsigned int t_f910_B : 1;
            unsigned int t_f910_identifier : 3;
        };

        struct {
            unsigned int : 8;
            unsigned int t_f11_Rd : 3;
            unsigned int : 1;
            unsigned int t_f11_identifier : 4;
        };

        struct {
            unsigned int : 8;
            unsigned int t_f12_Rd : 3;
            unsigned int t_SP: 1;
            unsigned int t_f12_identifier : 4;
        };

        struct {
            unsigned int t_SWord7 : 7;
            unsigned int t_f13_S : 1;
            unsigned int t_f13_identifier : 8;
        };

        struct {
            unsigned int t_Rlist : 8;
            unsigned int t_f14_R : 1;
            unsigned int t_f14_identifier1 : 2;
            unsigned int : 1;
            unsigned int t_f14_identifier2 : 4;
        };

        struct {
            unsigned int : 8;
            unsigned int t_f15_Rb : 3;
            unsigned int : 1;
            unsigned int t_f15_identifier : 4;
        };

        struct {
            int t_Soffset8 : 8;
            unsigned int t_Cond : 4;
            unsigned int t_f16_identifier : 4; 
        };

        struct {
            unsigned int t_Value : 8;
            unsigned int t_f17_identifier : 8;
        };

        struct {
            int t_Offset11 : 11;
            unsigned int t_f18_identifier : 5;
        };

        struct {
            unsigned int t_Offset : 11;
            unsigned int : 1;
            unsigned int t_f19_identifier : 4;
        };
        
    };

    struct Instruction {
        InstructionMnemonic mnemonic;
        InstructionAttributes attributes;
        u32 address;
        u16 format;
    };

    enum ShiftOp {
        lsl, lsr, asr, rr
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

        //Pipeline
    struct CpuPipeline {
        u32 fetch_stage;
        Instruction decode_stage;
    };

    struct Core {

        RegisterFile rf;
        CpuPipeline Pipeline;
        // u32 pipeline[2];
        u32 (*MMU)(MemOp mem_op);

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

    void SetMMU(Core& cpu, void* func_ptr);
    void Reset(Core& cpu);
    
    void Flush(Core& cpu, bool fetch=true);
    u32 Fetch(Core& cpu, State current_state);
    void Execute(Core& cpu, Instruction inst);

    void Step(Core& cpu);
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