#include "../include/Sumi/window.hpp"
#include "../include/Sumi/arm.hpp"
#include <iostream>
#include <iomanip>
#include <string>
#include <cstdio>


#define HEX( x ) std::setw(2) << std::setfill('0') << std::hex << (int)( x )

int main() {
    //probably initialize state before we enter rendering loop. 
    unsigned char MEM[0x10];
    Arm arm = Arm();
    arm.load_rom(MEM, std::string("roms/CPUTest.gba"), 0x10);
    for(unsigned char byte : MEM){
        std::cout << HEX(byte) << " ";       
    }
    return 0;
    // return run_app();
}
