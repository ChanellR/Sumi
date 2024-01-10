#ifndef EMULATOR_H
#define EMULUTOR_H

#include <CPU.hpp>
#include <Memory.hpp>
#include <GPU.hpp>
#include <utils.hpp>

class Emulator 
{
public:
    CPU cpu;
    GPU gpu;
    Memory memory; 
    bool running;

    Emulator() : cpu(memory), gpu(memory), running(false) {}
    ~Emulator() {}

    void Update() 
    {
        if(running)
            RunFrame();
    }

    void Reset()
    {
        cpu.Reset();
        memory.Reset();
        // gpu.Reset();
    }

    void Step() 
    {
        cpu.Step();
    }

    void RunFrame()
    {
        // const size_t Limit = 0x100;
        // while (cpu.cycles < Limit) {
        //     cpu.Step();
        // }
        // cpu.cycles = 0;
        for (int i = 0; i < 0x100; i++)
            cpu.Step();
    }

    void Pause() 
    {
        running = !running;
    }

    void LoadRom(std::string file_path)
    {
        memory.LoadRom(file_path);
    }

    void Display()
    {
        gpu.draw_bit_map();
        GL_CHECK(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 240, 160, 0, GL_RGBA, GL_FLOAT, gpu.FrameBuffer));
    }
};

#endif
