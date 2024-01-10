#include <Memory.hpp>
#include <arm.hpp>

uint32_t Memory::access(MemOp mem_op)
{

    uint32_t addr = mem_op.addr;
    uint8_t* location = nullptr;
    
    if(addr >= 0x00000000 && addr <= 0x00003FFF){
        //BIOS
        //some type of handling
    }
    else if(addr >= 0x00004000 && addr <= 0x01FFFFFF){
        //not used
    }
    else if(addr >= 0x02000000 && addr <= 0x0203FFFF){
        //board_RAM
        location = &bus.board_RAM[addr-0x02000000];
    }
    else if(addr >= 0x02040000 && addr <= 0x02FFFFFF){
        //NU
    }
    else if(addr >= 0x03000000 && addr <= 0x03007FFF){
        //chip_RAM
        location = &bus.chip_RAM[addr-0x03000000];
    }
    else if(addr >= 0x03008000 && addr <= 0x03FFFFFF){
        //NU
    }
    else if(addr >= 0x04000000 && addr <= 0x040003FE){
        //IO
        location = &bus.IO[addr-0x04000000];
        if(addr == DISPSTAT) {
            *location ^= 0x1;
            return *location; 
        }
    }
    else if(addr >= 0x04000400 && addr <= 0x04FFFFFF){
        //NU
    }
    else if(addr >= 0x05000000 && addr <= 0x050003FF){
        //Palette
        location = &bus.palette[addr-0x05000000];
    }
    else if(addr >= 0x05000400 && addr <= 0x05FFFFFF){
        //NU
    }
    else if(addr >= 0x06000000 && addr <= 0x06017FFF){
        //VRAM
        location = &bus.VRAM[addr-0x06000000];
    }
    else if(addr >= 0x06018000 && addr <= 0x06FFFFFF){
        //NU
    }
    else if(addr >= 0x07000000 && addr <= 0x070003FF){
        //OAM
        location = &bus.OAM[addr-0x07000000];
    }
    else if(addr >= 0x07000400 && addr <= 0x07FFFFFF){
        //NU
    }
    else if(addr >= 0x08000000 && addr <= 0x09FFFFFF){
        //ROM Wait 0
        location = &bus.ROM[addr-0x08000000];
    }
    else if(addr >= 0x0A000000 && addr <= 0x0BFFFFFF){
        //ROM Wait 1
        location = &bus.ROM[addr-0x0A000000];
    }
    else if(addr >= 0x0C000000 && addr <= 0x0DFFFFFF){
        //ROM Wait 2
        location = &bus.ROM[addr-0x0C000000];
    }
    else if(addr >= 0x0E000000 && addr <= 0x0E00FFFF){
        //SRAM 
        location = &bus.SRAM[addr-0x0E000000];
    }
    else if(addr >= 0x0E010000 && addr <= 0x0FFFFFFF){
        //NU
    }
    else if (addr >= 0x10000000 && addr <= (uint32_t)-1){
        //NU
        // printf("Reached unreachable memory!");  
        // exit(0);
    }
    if (location == nullptr) {
        return 0;
    }

    uint32_t load_val;

    //Little-endian
    switch(mem_op.operation){
        case ldb: {
            load_val = *location;
            break;
        }
        case ldh: {
            load_val = *location;
            load_val |= *(location + 1) << 8;
            break;
        }
        case ldsb: {
            load_val = (uint32_t)(int8_t)*location;
            break;
        }
        case ldsh: {
            load_val = *location;
            load_val |= (uint32_t)((int8_t)*(location + 1)) << 8;
            break;
        }
        case ldw: {
            load_val = *location;
            load_val |= *(location + 1) << 8;
            load_val |= *(location + 2) << 16;
            load_val |= *(location + 3) << 24;
            break;
        }
        case strb: {
            *location = (uint8_t)mem_op.data;
            break;
        }
        case strh: {
            *location = (uint8_t)(mem_op.data & 0xff);
            *(location + 1) = (uint8_t)((mem_op.data & 0xff00) >> 8);
            break;
        }
        case strw: {
            *location = (uint8_t)(mem_op.data & 0xff);
            *(location + 1) = (uint8_t)((mem_op.data & 0xff00) >> 8);
            *(location + 2) = (uint8_t)((mem_op.data & 0xff0000) >> 16);
            *(location + 3) = (uint8_t)((mem_op.data & 0xff000000) >> 24);
            break;
        }
    }

    return load_val;
}