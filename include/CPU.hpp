#ifndef CPU_H_
#define CPU_H_

#include <arm.hpp>
#include <Memory.hpp>

class CPU
{
public:  

    size_t cycles = 0; 

    CPU(Memory& memory) : core(memory)
    {
        // ARM::SetMMU(core, (void*)&memory.access);
        ARM::Reset(core);
    }

    ~CPU() {}

    void Reset()
    {
        ARM::Reset(core);
    }

    void Step() 
    {
        // Perhaps Step should return the amount of cycles?
        cycles += ARM::Step(core);   
    }

    void SetReg(u16 reg, u32 value)
    {
        ARM::SetReg(core, reg, value);
    }

private:
    // need some way to gauge progress in computing, set frame limits
    // will probably have to set up some cycle tables for things
    ARM::Core core;
};

#endif