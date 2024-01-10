#ifndef GPU_H
#define GPU_H

#include <Memory.hpp>
#include <arm.hpp>

class GPU 
{
public:

    GPU(Memory& memory) : mem(memory) {}
    ~GPU() {}

    float FrameBuffer[240 * 160 * 4]; //bitmap (4 bytes per pixel)

    void draw_bit_map(){

        /*
            BG Mode 4 - 240x160 pixels, 256 colors (out of 32768 colors)
            One byte is associated to each pixel, selecting one of the 256 palette entries.
            Color 0 (backdrop) is transparent, and OBJs may be displayed behind the bitmap.
            The first 240 bytes define the topmost line, the next 240 the next line, and so on.
            The background occupies 37.5 KBytes,
            allowing two frames to be used (06000000-060095FF for Frame 0, and 0600A000-060135FF for Frame 1).

            Color Definitions
            Each color occupies two bytes (same as for 32768 color BG modes):
            Bit   Expl.
            0-4   Red Intensity   (0-31)
            5-9   Green Intensity (0-31)
            10-14 Blue Intensity  (0-31)
            15    Not used
        */
        
        union Color {
            uint16_t palette_val;
            struct {
                unsigned int Red : 5; //(0-31)
                unsigned int Green : 5;
                unsigned int Blue : 5;
                int : 1;
            };
        };

        Color color {0};
        //The BG palette is just and array of 2byte colors that is 256 elements long
        for(uint32_t pixel = 0; pixel < 240 * 160; pixel++){
    
            uint8_t palette_num = mem.bus.VRAM[pixel];
            color.palette_val = ((uint16_t)(mem.bus.palette[palette_num * 2 + 1]) << 8) | (mem.bus.palette[palette_num * 2]);

            FrameBuffer[(pixel * 4)] = (color.Red / 31.0f);
            FrameBuffer[(pixel * 4) + 1] = (color.Green / 31.0f);
            FrameBuffer[(pixel * 4) + 2] = (color.Blue / 31.0f);
            FrameBuffer[(pixel * 4) + 3] = 1.0f; //alpha

        }
    }

private:
    Memory& mem;
};

#endif
