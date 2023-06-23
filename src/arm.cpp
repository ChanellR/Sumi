#include "../include/Sumi/arm.hpp"
#include <cstdio>
#include <iostream>
#include <string>
#include <stdint.h>
#include <assert.h>

#include <cstring>

template <typename F>
void repeat(unsigned n, F f) {
    while (n--) f();
}

void Arm::reset(){
    //sets all registers to 0
    memset(rf, 0, 4*17);
    //for now
    rf[PC] = 0x08000000; //beginning of ROM
    rf[LR] = 0x08000000; //beginning of ROM
    rf[SP] = 0x03007F00;
    rf[CPSR] = 0x0000001F; //user mode
}

void Arm::info(uint32_t instruction){
    Arm::InstructionAttributes inst;
    inst.word = instruction;
    char Disassembly[20];
    InstructionMnemonic mnemonic = decode(instruction);
    DisplayTypes type = get_display_type(mnemonic);
    disassemble(Disassembly, inst.word);
    printf("instruction: 0x%X %s\n",instruction, Disassembly);
    printf("mnemonic: %d display type: %d\n", mnemonic, type);
    printf("Imm?: %X type: 0x%X reg?: %u shift_reg:%u shift_amount:0x%X base_reg: %u \n",
    inst.dp_I, inst.dp_shift_type, inst.dp_reg_shift, inst.dp_Rs, inst.dp_shift_amount, inst.dp_Rm);
}

uint32_t Arm::fetch(){
    //just fetching double-words for now
    MemOp fetch_operation = {rf[PC], ldw};
    uint32_t instruction = MMU(fetch_operation);
    rf[PC] += 4;
    return instruction;
}

bool Arm::condition_pass(ConditionField condition, bool print_out){
    PSR cpsr;
    cpsr.word = rf[CPSR];
    bool outcome;
    switch (condition)
    {
        case EQ: outcome = cpsr.Z; break;
        case NE: outcome = !cpsr.Z;  break;
        case CS: outcome = cpsr.C;  break;
        case CC: outcome = !cpsr.C;  break;
        case MI: outcome = cpsr.N;  break;
        case PL: outcome = !cpsr.N;  break;
        case VS: outcome = cpsr.V;  break;
        case VC: outcome = !cpsr.V;  break;
        case HI: outcome = cpsr.C && !cpsr.Z;  break;
        case LS: outcome = !cpsr.C || cpsr.Z;  break;
        case GE: outcome = cpsr.N == cpsr.V;  break;
        case LT: outcome = cpsr.N != cpsr.V;  break;
        case GT: outcome = !cpsr.Z && (cpsr.N == cpsr.V);  break;
        case LE: outcome = cpsr.Z || (cpsr.N != cpsr.V);  break;
        case AL: outcome = true;  break;
        default:
        outcome = false;
    }
    // if(rf[PC] >= 0x08000D04 && rf[PC] <= 0x08000D20) printf("bool: %i conditition: %d\n", outcome, condition);
    return outcome;
}

void Arm::set_reg(uint16_t reg, uint32_t value){
    rf[reg] = value;
}

Arm::InstructionMnemonic Arm::decode(uint32_t instruction){

    InstructionAttributes inst;
    inst.word = instruction;

    //TODO;
    //MUL, SWP, Coprocessor
    if(inst.ldstM_valid == 0B100) return (inst.ldstM_L) ? LDM : STM;
    
    if(MATCH(instruction & 0xe0000010, 0x60000010)) return UNDEF;
    if(MATCH(instruction & 0x0ffffff0, 0x012FFF10)) return BX;
    if(MATCH(instruction & 0x0F000000, 0x0F000000)) return SWI;
    if (MATCH(instruction & 0x0E000000, 0x0A000000)) return B;

    if(inst.ldst_identifier == 0B01){
        return (inst.ldst_L) ? LDR : STR;
    }
    
    //Halfword and Signed Data Transfer && SWP && MUL
    if (inst.ldstx_valid_3 == 0 &&  inst.ldstx_valid_2 && inst.ldstx_valid_1) {
        if(inst.ldstx_SH == 0){
            //MUL && SWP
            if(MATCH(instruction & 0x0FB00FF0, 0x01000090)){
                return SWP;
            }
        } else if(inst.ldstx_L){
            //load
            if(!inst.ldstx_immediate_type && inst.ldstx_offset_2 == 0) {
                //register offset
                switch(inst.ldstx_SH){
                    case 0B01: return LDRH;
                    case 0B10: return LDRSB; 
                    case 0B11: return LDRSH;
                }
            } else if (inst.ldstx_immediate_type) {
                //immediate offset
                switch(inst.ldstx_SH){
                    case 0B01: return LDRH;
                    case 0B10: return LDRSB; 
                    case 0B11: return LDRSH;
                }
            }

        } else {
            //store
            if(!inst.ldstx_immediate_type && inst.ldstx_offset_2 == 0) {
                //register offset
                return STRH;
            } else if (inst.ldstx_immediate_type) {
                //immediate offset
                return STRH;
            }
        }
    }

    if(inst.dp_valid == 0){
        //checks for Immediate offset or register with proper_validation_bits
        //0B1110_00_1_1010_1_0010_0000_0000_00111001 
        // printf("Imm_type: %u\n", inst.dp_I);
        if(inst.dp_I || ((!inst.dp_reg_padding_bit && inst.dp_reg_shift) || !inst.dp_reg_shift)){
            InstructionMnemonic prelim_type = (InstructionMnemonic)inst.dp_OpCode;
            // printf("prelim_type: %u\n", prelim_type);
            // printf("S: %d\n", inst.dp_S);
            // 0xE128F00A
            //0B1110_00_0_10_0_1010001111_00000000_1010
            if(prelim_type == TEQ || prelim_type == TST || prelim_type == CMN || prelim_type == CMP){
                if(!inst.dp_S){
                    if(MATCH(instruction & 0x0FBF0FFF, 0x010F0000)){
                        //transfer PSR contents to a register
                        return MRS;
                    } else if (MATCH(instruction & 0x0FBFFFF0, 0x0129F000)){
                        //transfer register contents to PSR
                        return MSR;
                    } else if (MATCH(instruction & 0x0DBFF000, 0x0128F000)){
                        //transfer register contents or immediate value 
                        //to flag bits only
                        return MSRf;
                    } 
                } 

            } 
            return prelim_type;
        }
    }

    return UNIMP;
}

void Arm::disassemble(char* buffer, uint32_t instruction, uint32_t instruction_addr){
    //for translating into readable syntax
    InstructionMnemonic mnemonic = decode(instruction);
    DisplayTypes type = get_display_type(mnemonic);

    // printf("mnemonic: %d display type: %d\n", mnemonic, type);
    InstructionAttributes inst;
    inst.word = instruction;


#define HEADERBEGIN 0x08000000
#define HEADEREND 0x080000C0
    if(instruction_addr > HEADERBEGIN && instruction_addr < HEADEREND) {
        sprintf(buffer, "dd");
        return;
    }

    //will add more later
    const char* instruction_stems[] {
        "AND","EOR","SUB","RSB","ADD","ADC","SBC","RSC",
        "TST","TEQ","CMP","CMN","ORR","MOV","BIC","MVN",
        "B","UNDEF","UNIMP", "MRS", "MSR", "BX", "LDR",
        "STR", "SWI", "LDRH", "STRH", "LDRSB", "LDRSH", 
        "LDM", "STM", "MSRf", "SWP"
    };

    const char* conditions_extensions[] {
        "EQ", //equal
        "NE", //not equal
        "CS", //unsigned higher or same
        "CC", //unsigned lower
        "MI", //negative
        "PL", //positive or zero
        "VS", //overflow
        "VC", //no overflow
        "HI", //unsigned higher
        "LS", //unsigned lower or same
        "GE", //greater or equal
        "LT", //less than
        "GT", //greater than
        "LE", //less than or equal
        "", //always
    
    };
    // return;
    const char* shift_type[]{"lsl", "lsr", "asr", "rr"};
    switch(type){
        case dp_1:{
            uint16_t Rd = inst.dp_Rd;
            uint16_t Rm = inst.dp_Rm;
            if(mnemonic == MSR) Rd = CPSR;
            if(mnemonic == MRS) Rm = CPSR;

            if(!inst.dp_I){
                if (inst.dp_reg_shift){

                    sprintf(buffer, "%s%s%s r%u, r%u, %s r%u",
                        instruction_stems[mnemonic],
                        conditions_extensions[inst.Cond],
                        (inst.dp_S) ? "S" : "",
                        Rd, 
                        Rm,
                        shift_type[inst.dp_shift_type],
                        inst.dp_Rs,
                        get_operand_2(inst)
                        );
                } else {

                    sprintf(buffer, "%s%s%s r%u, r%u, %s %Xh=%Xh",
                        instruction_stems[mnemonic],
                        conditions_extensions[inst.Cond],
                        (inst.dp_S) ? "S" : "",
                        Rd, 
                        Rm,
                        shift_type[inst.dp_shift_type],
                        inst.dp_shift_amount,
                        get_operand_2(inst)
                        );
                }
            } else {
                sprintf(buffer, "%s%s%s r%u, %Xh",
                    instruction_stems[mnemonic],
                    conditions_extensions[inst.Cond],
                    (inst.dp_S) ? "S" : "",
                    Rd, get_operand_2(inst));
            }
            break;
        }
        case dp_2:{
            if(!inst.dp_I && inst.dp_Shift == 0){
                sprintf(buffer, "%s%s r%u, r%u",
                instruction_stems[mnemonic],
                conditions_extensions[inst.Cond],
                inst.dp_Rn, inst.dp_Rm);
            } else {
                sprintf(buffer, "%s%s r%u, %Xh",
                instruction_stems[mnemonic],
                conditions_extensions[inst.Cond],
                inst.dp_Rn, get_operand_2(inst));
            }
            break;
        }
        case dp_3:{
            if(!inst.dp_I ){
                sprintf(buffer, "%s%s%s r%u, r%u, r%u, %s %Xh",
                instruction_stems[mnemonic],
                conditions_extensions[inst.Cond],
                (inst.dp_S) ? "S" : "",
                inst.dp_Rd, inst.dp_Rn, inst.dp_Rm,
                shift_type[inst.dp_shift_type],
                inst.dp_shift_amount
                );
            } else {
                sprintf(buffer, "%s%s%s r%u, r%u, %Xh",
                instruction_stems[mnemonic],
                conditions_extensions[inst.Cond],
                (inst.dp_S) ? "S" : "",
                inst.dp_Rd, inst.dp_Rn, get_operand_2(inst));
            }
            break;
        }
        case b_1: {
            sprintf(buffer, "B%s%s pc, %0Xh =%0Xh %s", //off by one sometimes, PIPELINE!
                    (inst.branch_link) ? "L" : "",
                    conditions_extensions[inst.Cond],
                    inst.branch_offset,
                    (uint32_t)(instruction_addr + (inst.branch_offset << 2) + 8),
                    (condition_pass((ConditionField)inst.Cond)) ? "true;" : "false;"
                ); 
            break;
        }
        case b_2:{
            if(rf[inst.branch_Rn] & 0x1){
                sprintf(buffer, "BX%s [r%u] =%Xh THUMB",
                conditions_extensions[inst.Cond],
                 inst.branch_Rn, rf[inst.branch_Rn]);
            } else {
                sprintf(buffer, "BX%s [r%u] =%Xh ARM",
                conditions_extensions[inst.Cond],
                 inst.branch_Rn, rf[inst.branch_Rn]);
            }
            break;
        }
        case ldst: {
            buffer += sprintf(buffer, "%s%s%s r%u, [r%u, %s%dh]", 
                    instruction_stems[mnemonic],
                    conditions_extensions[inst.Cond],
                    (inst.ldst_B && (mnemonic == STR || mnemonic == LDR)) ? "B" : "",
                    inst.ldst_Rd,
                    inst.ldst_Rn,
                    (inst.ldst_U) ? "" : "-",
                    get_mem_address(inst, instruction_addr, true)
                );
            if(rf[PC] <= instruction_addr + 2) sprintf(buffer, "=%08Xh",
                     (inst.ldst_P) ? get_mem_address(inst, instruction_addr) :
                     rf[inst.ldst_Rn]);
            break;
        }
        case ldstM: {
            sprintf(buffer, "%s%s r%u%s, {0x%X}",
                instruction_stems[mnemonic],
                conditions_extensions[inst.Cond],
                inst.ldstM_Rn,
                (inst.ldstM_W) ? "!" : "",
                inst.ldstM_register_list
            );
            break;
        }
        case swp:{
            sprintf(buffer, "%s%s%s r%u, r%u, [r%u]",
                instruction_stems[mnemonic],
                conditions_extensions[inst.Cond],
                (inst.ldst_B) ? "B" : "",
                inst.ldst_Rd, 
                inst.ldst_Rn,
                inst.ldst_Rm
            );
        }
        case NOOP: {
            sprintf(buffer, "NOOP"); 
            break;
        }
    }
}

void Arm::execute(uint32_t instruction, InstructionMnemonic mnemonic){

    //need to look at flags
    if(!condition_pass((ConditionField)CONDITION(instruction))) return;

    InstructionAttributes inst;
    inst.word = instruction;

    //Branching and Data Transfers
    switch(mnemonic){
        case B: {
            //why am I 4 off? I guess, read the thing -> PIPELINE!
            //I already step so I don't need to add another 4
            if(inst.branch_link) rf[LR] = rf[PC];
            int32_t offset = inst.branch_offset << 2;
            rf[PC] = (uint32_t)(rf[PC] + offset + 4);
            return;
        }
        case BX: {
            //Swap Modes
            rf[PC] = rf[inst.branch_Rn]; //first register bits[3:0]
            return;
        }
        //Memory Transfers
        case LDRH: case STRH: case LDRSB: case LDRSH: case LDR: case STR: {

            MemOp transfer_instruction;
            uint32_t addr = get_mem_address(inst, rf[PC] - 0x4);
            
            OpType op = (OpType)((inst.ldst_L << 1) | inst.ldst_B);
            
            switch(mnemonic){
                case LDRH: op = ldh; break; 
                case STRH: op = strh; break;
                case LDRSB: op = ldsb; break;
                case LDRSH: op = ldsh; break;
            }
            // printf("op: %d\n", op);

            transfer_instruction.operation = op;
            transfer_instruction.data = rf[inst.ldst_Rd]; //for stor operation

            if(inst.ldst_W && inst.ldst_P) { //may not be functioning correctly
                //pre
                rf[inst.ldst_Rn] = addr;
                transfer_instruction.addr = rf[inst.ldst_Rn];
            } else {
                transfer_instruction.addr = rf[inst.ldst_Rn];
            }
            // printf("r1 before: %n", rf[1]);
            uint32_t data = MMU(transfer_instruction);
            // printf("r1 after: %X\n", rf[1]);
            if(!inst.ldst_P) { //may not be functioning correctly
                rf[inst.ldst_Rn] = addr;
            } 
            
            if(mnemonic != STR && mnemonic != STRH){
                rf[inst.ldst_Rd] = data;
            }
            // printf("r1 changes: %X\n", rf[1]);
            return;
        }
        case LDM: case STM: {
            //what registers
            uint32_t base = rf[inst.ldstM_Rn];
            MemOp transfer {base, (inst.ldstM_L) ? ldw : strw, 0};
            if(inst.ldstM_U){
                for(int reg = 0; reg < 16; reg++){
                    if(TESTBIT(inst.ldstM_register_list, reg)){
                        //transfer from the where Rn is pointing for this register
                        transfer.data = rf[reg];
                        if(inst.ldstM_P){
                            transfer.addr += 0x4;
                            uint32_t out = MMU(transfer);
                            if(inst.ldstM_L) rf[reg] = out;
                        } else {
                            uint32_t out = MMU(transfer);
                            if(inst.ldstM_L) rf[reg] = out;
                            transfer.addr += 0x4;
                        }
                    }
                }
                if(inst.ldstM_W) rf[inst.ldstM_Rn] = transfer.addr;
            } else {
                for(int reg = 15; reg >= 0; reg--){
                    if(TESTBIT(inst.ldstM_register_list, reg)){
                        //transfer from the where Rn is pointing for this register
                        transfer.data = rf[reg];
                        if(inst.ldstM_P){
                            transfer.addr -= 0x4;
                            uint32_t out = MMU(transfer);
                            if(inst.ldstM_L) rf[reg] = out;
                        } else {
                            uint32_t out = MMU(transfer);
                            if(inst.ldstM_L) rf[reg] = out;
                            transfer.addr -= 0x4;
                        }
                    }
                }
                if(inst.ldstM_W) rf[inst.ldstM_Rn] = transfer.addr;
            }
            return;
        }
        case SWP:{
            MemOp load {rf[inst.ldst_Rn], (inst.ldst_B) ? ldb : ldw, 0};
            MemOp store {rf[inst.ldst_Rn], (inst.ldst_B) ? strb : strw, rf[inst.ldst_Rm]};
            rf[inst.ldst_Rd] = MMU(load);
            MMU(store);
            return;
        }
    }


    //(Rd <- int, Carry value) 
    std::pair<uint32_t, uint8_t> outcome;
    outcome = get_operand_2(inst);
    long long result = 0;
    bool logical = false;

    // logical: (AND, EOR, TST, TEQ, ORR, MOV, BIC, MVN)
    bool maintain_register = ((uint16_t)mnemonic >= 8 && (uint16_t)mnemonic <= 11);
    uint32_t Rn = rf[inst.dp_Rn];
    int subtraction = 0;
    //Data Processing
    //SUB things
    switch(mnemonic){
        case CMP: case SUB: case SBC:{
            subtraction = 1;
            outcome.first = ~outcome.first;
            outcome.second = (mnemonic == SBC) ? outcome.second : 1;
            mnemonic = ADC; 
            break;
        }
        case RSB: case RSC:{
            subtraction = 2;
            Rn = ~Rn;
            outcome.second = (mnemonic == RSC) ? outcome.second : 1;
            mnemonic = ADC; 
            break;
        }
    }

    switch (mnemonic)
    { 
        case MSR: case MSRf: { //there is a slight differnce in the docs considering bit16!!
            uint32_t result;
            if(!inst.dp_I && inst.dp_Shift == 0){
                result = rf[inst.dp_Rm];
            } else result = outcome.second;
            if(mnemonic == MSRf){
                rf[CPSR] &= ~(0xF0000000);
                rf[CPSR] |= result & (0xF0000000);
            } else rf[CPSR] = result;
            return;
        }
        case MRS: rf[inst.dp_Rd] = rf[CPSR]; return;
        //changes flags
        //Logical
        case TST: case AND: result = Rn & outcome.first; logical = true; break;
        case TEQ: case EOR: result = Rn ^ outcome.first; logical = true; break;
        case MOV: result = outcome.first; logical = true; break;
        case MVN: result = 0xFFFFFFFF ^ outcome.first; logical = true; break;
        case BIC: result = Rn & ~outcome.first; logical = true; break;
        case ORR: result = Rn | outcome.first; logical = true; break;
        //Arithmetic
        case CMN: case ADD: result = Rn + outcome.first; break;
        case ADC: {
            result = Rn; 
            result += outcome.first;
            result += outcome.second; 
            break;
        }
        // case CMP: case SUB: result = Rn - outcome.first; break;
        // case RSB: result = outcome.first - Rn; break;
        // case SBC: result = Rn - outcome.first + outcome.second; break;
        // case RSC: result = outcome.first - Rn + outcome.second; break;
        case UNIMP: {
            return;
        }
    }

    if(!maintain_register) rf[inst.dp_Rd] = (uint32_t)result;
    //set status flags
    // printf("result: %u\n", (result));
    if(inst.dp_S && inst.dp_Rd != 15) { //check for other things too when apparent
        PSR cpsr;
        cpsr.word = rf[CPSR];

        if(logical){
            // printf("in\n");
            cpsr.C = outcome.second;

            cpsr.Z = ((uint32_t)result == 0) ? 1 : 0;
            // printf("out.sec: %X\n", outcome.second);
            cpsr.N = ((uint32_t)result & (0x1 << 31)) ? 1 : 0;
        } else {
            //I don't know if they are reset after unsuccessful instructions or not 

            // printf("N: %u\n", cpsr.N);
            // printf("result: %X\n", result);
            //int32_t bounds

            cpsr.V = ((int32_t)result > 2147483647 || (int32_t)result < -2147483647) ? 1 : 0;
            // cpsr.V = (Rn & N_FIELD);

            uint32_t a, b, c;
            a = Rn;
            b = outcome.first;
            c = outcome.second;
            // cpsr.C = (())
            long long x = 0x1A6143389;
            // cpsr.C = (result > (long long)0xFFFFFFFF) ? 1 : 0; //bit 31 carry
            // cpsr.C = (a & (0x1 << 31) && b & (0x1 << 31)) || 
            switch (subtraction)
            {
                case 0:{
                    cpsr.C = ((0xFFFFFFFF - Rn) < outcome.first);
                    break;
                }
                case 1:{
                    cpsr.C = (Rn >= ~outcome.first);
                    // cpsr.N = (!((Rn ^ ~outcome.first) & (1 << 31)) && !((Rn ^ outcome.second) & (1 << 31)));
                    break;
                }
                case 2:{
                    cpsr.C = (outcome.first >= ~Rn);
                    break;    
                }
            }
            cpsr.Z = ((uint32_t)result == 0) ? 1 : 0;
            cpsr.N = ((uint32_t)result & (0x1 << 31)) ? 1 : 0;

            // cpsr.word &= ~(0xF0000000);
            // cpsr.word |= (a & 0x80000000);
            // cpsr.word |= (uint32_t)(a == 0) << 30;
            // cpsr.word |= (uint32_t)((0xFFFFFFFF - a) < b) << 29;
            // cpsr.word |= (uint32_t)(!((a ^ b) & (1 << 31)) && ((a ^ c) & (1 << 31))) << 28;

        }
        // printf("after calcs: %X\n", cpsr.word);
        rf[CPSR] = cpsr.word;
    }
}

// private void setFlagsSub(const u32 a, const u32 b, const u32 c) {
//     regs.psrs[PSR.cpsr] &= ~(PSRField.n | PSRField.z | PSRField.c | PSRField.v);

// regs.psrs[PSR.cpsr] |= (a & PSRField.n);
// regs.psrs[PSR.cpsr] |= cast(u32)(a == 0) << 30;
// regs.psrs[PSR.cpsr] |= cast(u32)(a >= b) << 29;
// regs.psrs[PSR.cpsr] |= cast(u32)(!((a ^ b) & (1 << 31)) && !((a ^ c) & (1 << 31))) << 28;
// }


void Arm::step(){
    uint32_t instruction = fetch();
    InstructionMnemonic instruction_type = decode(instruction);
    execute(instruction, instruction_type);
}

void Arm::register_dump (char* buffer) const{
    //make sure to free afterwards!
    char* dump_ptr = buffer;
    for(uint8_t reg = 0; reg < REGSIZE; reg++){
        if(reg < 10){
            dump_ptr += sprintf(dump_ptr, "r%d:  0x%08X\n", reg, rf[reg]);
        } else if(reg == 16){
            dump_ptr += sprintf(dump_ptr, "cpsr: 0x%08X\n", rf[reg]);
        } else if(reg == 17){
            dump_ptr += sprintf(dump_ptr, "spsr: 0x%08X\n", rf[reg]);
        } else {
            dump_ptr += sprintf(dump_ptr, "r%d: 0x%08X\n", reg, rf[reg]);
        } 
    }
}

void Arm::rom_dump(char* buffer){
    uint32_t pc = rf[PC];
    for (int word = -15; word < 15; word++){
        if((int32_t)(pc + (word*4)) < 0) continue;

        char mnemonic[15] {0};
        Arm::MemOp fetch_operation = {pc + (word*4), Arm::ldw};
        uint32_t instruction = MMU(fetch_operation);

        if(!(pc + (word*4) > 0x8000000 && pc + (word*4) < 0x80000C0)) {
            disassemble(mnemonic, instruction, (pc + (word*4))); 
        } else {
            //Cartidge Header        rf[inst.rd] = result;
            *mnemonic = 'd';
            *(mnemonic + 1) = 'd'; 
        } 

        if (word == 0) {
            buffer += sprintf(buffer, "0x%08X: 0x%08X  %s <-\n", pc + (4*word), instruction, mnemonic);
        } else {
            buffer += sprintf(buffer, "0x%08X: 0x%08X  %s\n", pc + (4*word), instruction, mnemonic);
        }
    }
}

void Arm::set_MMU(void* func_ptr){
    MMU = (uint32_t(*)(MemOp)) func_ptr;
}

uint32_t Arm::get_pc() const{
    return rf[PC];
}

uint32_t Arm::get_mem_address(InstructionAttributes inst, uint32_t instruction_addr, bool offset_only){
    uint32_t addr_offset = 0;
    /*
        Write-back must not be specified if R15 is specified as the base register (Rn). When
        using R15 as the base register you must remember it contains an address 8 bytes on
        from the address of the current instruction.
        R15 must not be specified as the register offset (Rm).
        When R15 is the source register (Rd) of a register store (STR) instruction, the stored
        value will be address of the instruction plus 12.
    */
    // assert(!(inst.ldst_W && inst.ldst_Rn == PC));
    // assert(inst.ldst_Rm != PC);

    if(inst.ldst_I){
        inst.ldst_I = 0;
        std::pair<uint32_t, uint8_t> out;
        out = get_operand_2(inst);
        addr_offset = out.second;
        inst.ldst_I = 1;
    } else {
        addr_offset = inst.ldst_Imm;
    }

    switch(decode(inst.word)){
        case LDRH: case STRH: case LDRSB: case LDRSH: {
            if(!inst.ldstx_immediate_type && inst.ldstx_offset_2 == 0) {
                //register offset
                addr_offset = rf[inst.ldstx_Rm];
            } else if (inst.ldstx_immediate_type) {
                //immediate offset
                addr_offset = (inst.ldstx_offset_2 << 4) | inst.ldstx_offset_1;
            }
        }
    }

    if(offset_only) return addr_offset;

    uint32_t base = (inst.ldst_Rn == PC) ? instruction_addr + 0x8 : rf[inst.ldst_Rn]; //8 for pipeline waiting
    uint32_t addr = (inst.ldst_U) ? base + addr_offset : base - addr_offset;
    return addr;
}

std::pair<uint32_t, uint8_t> Arm::get_operand_2(InstructionAttributes inst){

    PSR cpsr;
    cpsr.word = rf[CPSR];
    uint8_t carry_out = cpsr.C;

    uint32_t result = 0;

    //I don't know which instructions are RRX(extended) and RR0
    if(inst.dp_I){
        //rotated immediate operand 2 
        uint32_t imm = inst.dp_Imm;
        uint8_t rotations = inst.dp_Rotate * 2;
        for (uint8_t times = 0; times < rotations; times++){
            carry_out = imm & 0x1;
            imm >>= 1;
            imm |= (carry_out << 31);
        }
        result = imm;

    } else {

        //shifted/rotated register
        uint32_t reg_value = rf[inst.dp_Rm];
        // printf("%X\n", reg_value);
        uint8_t shifts;
        if(inst.dp_reg_shift && !inst.dp_reg_padding_bit){
            //register shift

            shifts = (uint8_t)(rf[inst.dp_Rs] & 0xFF);
        } else if(!inst.dp_reg_shift){
            shifts = inst.dp_shift_amount; //bit4 not set
            // printf("shifts:%d", shifts);
        } 

        // printf("calculated shifts: %u\n", shifts);

        if(shifts == 0) return std::pair<uint32_t, uint8_t>{reg_value, carry_out};

        switch(inst.dp_shift_type){
            case 0B00:{ //lsl
                // printf("here");
                reg_value <<= shifts - 1;
                carry_out = (reg_value & 0x80000000) >> 31;
                result = reg_value << 1;
                break;
            }
            case 0B01:{ //lsr
                reg_value >>= shifts - 1;
                carry_out = (reg_value & 0x1);
                result = reg_value >> 1;
                break;
            }
            case 0B10:{ //asr
                result = (uint32_t)(((int32_t)reg_value) >> shifts);
                if (shifts < 32) {
                    carry_out = (reg_value >> shifts - 1) & 0x1;
                } else carry_out = reg_value >> 31;
                break;
            }
            case 0B11:{ //rr
                for (uint8_t times = 0; times < shifts; times++){
                    carry_out = reg_value & 0x1;
                    reg_value >>= 1;
                    reg_value |= (carry_out << 31);
                }
                result = reg_value;
                break;
            }
            default:{ //impossible
                result = 0;
                break;
            }
        }
    }

    /*
        The arithmetic operations (SUB, RSB, ADD, ADC, SBC, RSC, CMP, CMN) treat each
        operand as a 32 bit integer (either unsigned or 2's complement signed, the two are
        equivalent). If the S bit is set (and Rd is not R15) the V flag in the CPSR will be set if
        an overflow occurs into bit 31 of the result; this may be ignored if the operands were
        considered unsigned, but warns of a possible error if the operands were 2's
        complement signed. The C flag will be set to the carry out of bit 31 of the ALU, the Z
        flag will be set if and only if the result was zero, and the N flag will be set to the value
        of bit 31 of the result (indicating a negative result if the operands are considered to be
        2's complement signed).
    */

    return std::pair<uint32_t, uint8_t>{result, carry_out};

    if(inst.dp_S && inst.dp_Rd != 15) {
        //set status flags
        cpsr.C = carry_out;
        //V ? 
        if (result & (0x1 << 31)) cpsr.V = 1;
        cpsr.Z = (result == 0) ? 1 : 0;
        cpsr.N = (result & (0x1 << 31)) ? 1 : 0;
        rf[CPSR] = cpsr.word;
    }
    
}

void Arm::dump_instructions_attributes(InstructionAttributes inst, DisplayTypes type){

    switch(type){
        case dp_1: case dp_2: case dp_3: {
            printf("Instruction: %08X\n", inst.word);
            printf("operand_2: 0x%03X\n", inst.dp_operand_2);
            printf("Rd: 0x%0X\n", inst.dp_Rd);
            printf("Rn: 0x%0X\n", inst.dp_Rn);
            printf("S: %X\n", inst.dp_S);
            printf("OpCode: 0x%0X\n", inst.dp_OpCode);
            printf("I: %X\n", inst.dp_I);
            printf("Cond: 0x%0X\n", inst.Cond);
        }
    }
}

Arm::DisplayTypes Arm::get_display_type(InstructionMnemonic mnemonic){

    switch(mnemonic){
        case MOV: case MVN: case MRS: case MSR: case MSRf: {
            return dp_1;
        }
        case TST: case TEQ: case CMP: case CMN:{
            return dp_2;
        }
        case AND: case EOR: case SUB: case RSB:
        case ADD: case ADC: case SBC: case RSC:
        case ORR: case BIC: {
            return dp_3;
        }
        case B: return b_1;
        case BX: return b_2;
        case LDR: case STR: case LDRH: case STRH: case LDRSB: case LDRSH: {
            return ldst;
        }
        case STM: case LDM: {
            return ldstM;
            break;
        }
        case SWP:{
            return swp;
        }
        default:
            return NOOP;
    }
}