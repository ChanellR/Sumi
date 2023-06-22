#pragma once
#include <stdint.h>
#include <string>

#define NOP -1
#define REGSIZE 17

#define SP 13
#define LR 14
#define PC 15
#define CPSR 16

#define BIT_N 31
#define BIT_Z 30 
#define BIT_C 29 
#define BIT_V 28 

#define BIT_L 24
#define BIT_S 20
#define BIT_loadStore 20

#define CONDITION( x ) (uint8_t)((x & 0xf0000000) >> 28)
//match the nibble starting at the bit given
#define MATCHNIBBLE(x, template, bit) !((x & (0xf << bit)) ^ (template << bit))
#define MATCH(x, template)  !((x) ^ template)
#define GETNIBBLE(x, bit) (uint8_t)((x & (0xf << bit)) >> bit)
#define TESTBIT(x, bit) (x & (0x1 << bit)) >> bit

class Arm {

    enum State {arm_mode, thumb_mode};

    //Program Status Registers
    union PSR  {
        uint32_t double_word;
        struct {
            uint8_t operating_mode : 5;
            State operating_state : 1; //Thumb/Arm
            uint8_t interrupt_disable : 2;
            uint32_t : 20;
            int V : 1; //overflow
            int C : 1; //Carry / Borrow / Extend
            int Z : 1; //Zero
            int N : 1; //Negative / Less Than
        };
    };

    struct ControlSignals {
        uint8_t BIGEND : 1;
    };

    enum InstructionMnemonic {
        AND, EOR, SUB, RSB, ADD, ADC, SBC, RSC,
        TST, TEQ, CMP, CMN, ORR, MOV, BIC, MVN,
        B, UNDEF, UNIMP, MRS, MSR, BX, LDR, STR,
        SWI, LDRH, STRH, LDRSB, LDRSH,
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
    
    public:
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
            uint32_t addr;
            OpType operation;
            uint32_t data; //on store
        };

    uint32_t rf[17]; //register file
    //I don't need to store memory here I believe, I need only a memory access handler
    
    uint32_t (*MMU)(Arm::MemOp mem_op);

    //todo:
    //interrupts
    //cycle timing
    //coprocessor

    uint32_t fetch();
    InstructionMnemonic decode(uint32_t instruction);
    bool condition_pass(ConditionField condition);
    void execute(uint32_t instruction, InstructionMnemonic mnemonic);
    
    public:

        //how they are written in ARM assembly
        enum DisplayTypes {
            dp_1, // ? Rd,<Op2>
            dp_2, // ? Rn,<Op2>
            dp_3, // ? Rd,Rn,<Op2>
            b_1,  // B{L} <expression>
            b_2,  // BX Rn
            ldst,
            NOOP, // ?
        }; 
        
        union InstructionAttributes { //instruction interface

            //could also sort this by tab
            uint32_t word;

            struct {
                int : 28;
                unsigned int Cond : 4;
            };

            struct {
                unsigned int branch_Rn : 4;
            };

            struct {
                int branch_offset : 24;
                unsigned int branch_link : 1;
                int : 3;
            };

            struct {
                unsigned int dp_operand_2 : 12;
                unsigned int dp_Rd : 4;
                unsigned int dp_Rn : 4;
                unsigned int dp_S : 1;
                unsigned int dp_OpCode : 4;
                unsigned int dp_I : 1;
                unsigned int dp_valid : 2;
            };

            struct {
                unsigned int dp_Rm : 4;
                unsigned int dp_Shift : 4;
            };

            struct {
                int : 4;
                unsigned int dp_reg_shift : 1;
                unsigned int dp_shift_type : 2;
                unsigned int dp_reg_padding_bit : 1;
                unsigned int dp_Rs : 4;
            };

            struct {
                int : 4;
                int : 3;
                unsigned int dp_shift_amount : 5;
            };

            struct {
                unsigned int dp_Imm : 8;
                unsigned int dp_Rotate : 4;
            };

            struct {
                unsigned int ldst_Imm : 12; //Offset
                unsigned int ldst_Rd : 4;
                unsigned int ldst_Rn : 4;
                unsigned int ldst_L : 1; //Load/Store bit
                unsigned int ldst_W : 1; //Write-back bit
                unsigned int ldst_B : 1; //Byte/Word bit
                unsigned int ldst_U : 1; //Up/Down bit
                unsigned int ldst_P : 1; //Pre/Post Indexing bit
                unsigned int ldst_I : 1; // Immediate Offset Toggle
            };

            struct {
                unsigned int ldst_Rm : 4;
                unsigned int ldst_Shift : 8;
            };

            struct {
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

            struct {
                unsigned int ldstx_offset_1 : 4;
            };
            
        };

        void dump_instructions_attributes(InstructionAttributes inst, DisplayTypes type);
        void reset();
        void step();
        void register_dump(char* buffer) const;
        void rom_dump(char* buffer);
        void set_MMU(void* func_ptr);
        uint32_t get_pc() const;
        void disassemble(char* buffer, uint32_t instruction, uint32_t instruction_addr = 0);
        void set_reg(uint16_t reg, uint32_t value);
    private:
        uint32_t get_operand_2(InstructionAttributes inst); //data processing
        uint32_t get_mem_address(InstructionAttributes inst, uint32_t instruction_addr = 0, bool offset_only=false);
        DisplayTypes get_display_type(InstructionMnemonic mnemonic);
};
    

