#include "../include/glad/glad.h"
#include "../include/GLFW/glfw3.h"
#include "../imgui/imgui.h"
#include "../imgui/imgui_impl_glfw.h"
#include "../imgui/imgui_impl_opengl3.h"

#define STB_IMAGE_IMPLEMENTATION
#include "../include/extra/stb_image.h"

#include <iostream>
#include <map>
#include <set>
#include <windows.h>
#include "../include/Sumi/window.hpp"

void FramebufferSizeCallback(GLFWwindow* window, int width, int height);
bool TestKeyPress(GLFWwindow *window, uint16_t key, std::map<uint16_t, uint8_t> &last_presses);
void OpenMemoryTable(GBA* gba_handle, uint32_t base_address);
static void ShowMainMenuBar(ARMCore* arm_handle, GBA* gba_handle);

// settings
const unsigned int SCR_WIDTH = 1000;
const unsigned int SCR_HEIGHT = 800;

EmulatorSettings settings {true, true, false, true, true, false, true, false, 0x05000000, true};
EmulatorData data {0};

int run_app(ARMCore* arm_handle, GBA* gba_handle)
{
    const char* glsl_version = "#version 130";

    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // glfw window creation
    // --------------------
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Sumi", NULL, NULL);

    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, FramebufferSizeCallback);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }    



    //imgui setup

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);
    
    arm_handle->RegisterDump(data.reg_dump_buffer);
    gba_handle->stack_dump(data.stack_dump_buffer, arm_handle->GetReg(SP));

    float test_image[20*20*4];

    GLuint renderedTexture = 0;
    int bitmap_width = 240;
    int bitmap_height = 160;

    glGenTextures(1, &renderedTexture);
    glBindTexture(GL_TEXTURE_2D, renderedTexture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    GLFWimage images[1]; 
    images[0].pixels = stbi_load("frisk.png", &images[0].width, &images[0].height, 0, 4); //rgba channels 
    glfwSetWindowIcon(window, 1, images); 
    stbi_image_free(images[0].pixels);


    while (!glfwWindowShouldClose(window))
    {
        ImGui_ImplGlfw_NewFrame();
        ImGui_ImplOpenGL3_NewFrame();
        ImGui::NewFrame();
        
        bool printpipeline = true;
        if(settings.running && !settings.enable_debug) {
            for(int inst = 0; inst < STEPSPERSEC; inst++){
                arm_handle->Step();
            }
           if(settings.enable_video)  gba_handle->draw_bit_map();
        } else if(settings.running && settings.enable_debug){
            printpipeline = false;
            for(int inst = 0; inst < STEPSPERSEC; inst++){
                arm_handle->Step();
                // if(arm_handle->Pipeline.fetch_stage == 0) {
                //     settings.running = false;
                //     break;
                // }
                if(data.breakpoints.count(arm_handle->GetReg(PC)) == 1) {
                    arm_handle->RegisterDump(data.reg_dump_buffer);
                    gba_handle->stack_dump(data.stack_dump_buffer, arm_handle->GetReg(SP));
                    settings.running = false;
                    break;
                }   
            }
            if(settings.enable_video) gba_handle->draw_bit_map();
        }
        
        ShowMainMenuBar(arm_handle, gba_handle);
        
        if(settings.enable_debug){
 
            if(settings.enable_reg_file){
                ImGui::Begin("Register Dump");
                ImGui::Text(data.reg_dump_buffer);
                ImGui::End();
            }

            if(settings.enable_instructions){
                ImGui::Begin("Instruction Memory");
                ImGui::BeginChild("Scrolling");
                ARM::State current_state = arm_handle->GetOperatingState();
                {
                    uint32_t pc = arm_handle->GetReg(PC);
                    for (int word = -10; word < 10; word++){
                        uint32_t addr = (pc + (word*((current_state) ? 2 : 4)));
                        if(addr < 0) continue;

                        char mnemonic[70];
                        ARM::MemOp fetch_operation {addr, (current_state) ? ARM::ldh : ARM::ldw};
                        uint32_t instruction = arm_handle->MMU(fetch_operation);

                        arm_handle->Disassemble(mnemonic, instruction, addr); 

                        if (addr == arm_handle->GetExecuteStageAddr()) {
                            ImGui::TextColored(ImVec4(0,1,0.6,0.9),"%08X | 0x%08X  %s", addr, instruction, mnemonic);
                        } else {
                            ImGui::Text("%08X | 0x%08X  %s", addr, instruction, mnemonic);
                        }
                    }
                }

                ImGui::EndChild();
                ImGui::End();
            }
            if(settings.enable_stack){
                ImGui::Begin("Stack");
                ImGui::BeginChild("Scrolling");
                ImGui::Text(data.stack_dump_buffer);
                ImGui::EndChild();
                ImGui::End();
            }

            ImGuiInputTextFlags input_text_flags = ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_CharsHexadecimal | ImGuiInputTextFlags_EscapeClearsAll ;
            
            if(settings.enable_memory){
                ImGui::Begin("Memory Viewer");
                ImGui::InputInt("Search", &settings.search_address, 0, 0, input_text_flags);
                OpenMemoryTable(gba_handle, settings.search_address);
                ImGui::End();
            }

            if(settings.enable_break_points){
                ImGui::Begin("Breakpoints");
                int new_break_point = (int)NULL;
                char* ptr = data.breakpoints_buffer;
                *ptr = 0;
                for(int breakpoint : data.breakpoints) ptr += sprintf(ptr, "%X\n", breakpoint);
                ImGui::InputInt("##", &new_break_point, 0, 0, input_text_flags);
                if(new_break_point) data.breakpoints.insert(new_break_point);
                ImGui::Text(data.breakpoints_buffer);
                ImGui::End();
            }

        }

        // break
        glfwPollEvents();
        
        if(settings.enable_debug){
            if(TestKeyPress(window, GLFW_KEY_F10, data.key_presses)) {
                //step debugger
                arm_handle->Step();
                arm_handle->RegisterDump(data.reg_dump_buffer);
                gba_handle->stack_dump(data.stack_dump_buffer, arm_handle->GetReg(SP));
                if(settings.enable_video) gba_handle->draw_bit_map();
            }
        } 
        
        if (settings.enable_controls) {
            // 4000130h - KEYINPUT - Key Status (R)
            // Bit   Expl.
            // 0     Button A        (0=Pressed, 1=Released)
            // 1     Button B        (etc.)
            // 2     Select          (etc.)
            // 3     Start           (etc.)
            // 4     Right           (etc.)
            // 5     Left            (etc.)
            // 6     Up              (etc.)
            // 7     Down            (etc.)
            // 8     Button R        (etc.)
            // 9     Button L        (etc.)
            // 10-15 Not used
            int gameboy_buttons[] {GLFW_KEY_K, GLFW_KEY_L, GLFW_KEY_N, GLFW_KEY_M,
                                    GLFW_KEY_D, GLFW_KEY_A, GLFW_KEY_W, GLFW_KEY_S,
                                    GLFW_KEY_W, GLFW_KEY_S, GLFW_KEY_P, GLFW_KEY_U};
            uint16_t keys_pressed = 0x1FF;
            for(int button = 0; button < 10; button++){
                if(glfwGetKey(window, gameboy_buttons[button]) == GLFW_PRESS) {
                    keys_pressed &= ~(0x1 << button); 
                    // if(button == 3) settings.running = false;
                    // printf("key_pressed: %i\n", button);
                }
            }
            ARM::MemOp store_keypad {KEYINPUT, ARM::strh, keys_pressed};
            gba_handle->memory_access(store_keypad);
        }


        if(TestKeyPress(window, GLFW_KEY_F9, data.key_presses))  settings.running = !settings.running;
        if(TestKeyPress(window, GLFW_KEY_ESCAPE, data.key_presses)) glfwSetWindowShouldClose(window, true);
        if(TestKeyPress(window, GLFW_KEY_F4, data.key_presses)) data.breakpoints.clear();
        if(TestKeyPress(window, GLFW_KEY_F5, data.key_presses)) {
            //Reset
            arm_handle->Reset(); 
            gba_handle->Reset();
            arm_handle->RegisterDump(data.reg_dump_buffer);
            gba_handle->stack_dump(data.stack_dump_buffer, arm_handle->GetReg(SP));
            gba_handle->draw_bit_map();
            settings.running = false;
        }

        //load bitmap texture
        glTexImage2D(
            GL_TEXTURE_2D, 0, GL_RGBA8,
            bitmap_width, bitmap_height, 0,
            GL_RGBA, GL_FLOAT, 
            gba_handle->FRAME
        );

        ImGui::Begin("BitMap Render");
        ImGui::Image((void*)(intptr_t)renderedTexture, ImVec2(bitmap_width*2, bitmap_height*2));
        ImGui::End();   
        
        glClearColor(0xC4/255.0f, 0xAA/255.0f, 0x7D/255.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)r
        // -------------------------------------------------------------------------------

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
        Sleep((1000/60.0f)); //60 fps cap   
        
    }

    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}

//single press key stroke testing
bool TestKeyPress(GLFWwindow *window, uint16_t key, std::map<uint16_t, uint8_t> &last_presses){
    uint8_t current_state = glfwGetKey(window, key);
    if(current_state == GLFW_RELEASE && last_presses[key] == GLFW_PRESS){
        last_presses[key] = current_state;
        return true;
    }
    last_presses[key] = current_state;
    return false;
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void FramebufferSizeCallback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}



void OpenMemoryTable(GBA* gba_handle, uint32_t base_address){

    static ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersOuter | ImGuiTableFlags_BordersV | ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable | ImGuiTableFlags_Hideable;

    // When using ScrollX or ScrollY we need to specify a size for our table container!
    // Otherwise by default the table will fit all available space, like a BeginChild() call.
    // ImVec2 outer_size = ImVec2(0.0f, 0.0f);
    ImGui::BeginTable("Memory Viewer", 2, flags);
    {
        ImGui::TableSetupScrollFreeze(0, 1); // Make top row always visible
        ImGui::TableSetupColumn("Address", ImGuiTableColumnFlags_None);
        ImGui::TableSetupColumn("Word", ImGuiTableColumnFlags_None);
        ImGui::TableHeadersRow();

        // Demonstrate using clipper for large vertical lists
        for (uint32_t row = base_address - (20*4); row < base_address + (20*4); row+=4)
        {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("0x%08X", row);
            ImGui::TableSetColumnIndex(1);
            ARM::MemOp op {row, ARM::ldw};
            if(row == base_address) {
                ImGui::TextColored(ImVec4(0,1,0.6,0.9), "0x%08X", gba_handle->memory_access(op));
            } else {
                ImGui::Text("0x%08X", gba_handle->memory_access(op));
            }
            
        }
    }
    ImGui::EndTable();

}

static void ShowMainMenuBar(ARMCore* arm_handle, GBA* gba_handle)
{
    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("File"))
        {
            if (ImGui::MenuItem("Open", "CTRL+Z")) {
                //need file dialog, or just a text box
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Emulation"))
        {
            if (ImGui::MenuItem("Reset", "F5")) {
                //Reset
                arm_handle->Reset(); 
                gba_handle->Reset();
                arm_handle->RegisterDump(data.reg_dump_buffer);
                gba_handle->stack_dump(data.stack_dump_buffer, arm_handle->GetReg(SP));
                gba_handle->draw_bit_map();
            }
            if (ImGui::MenuItem("Run", "F9")) {
                settings.running = !settings.running;
            }
            if (ImGui::MenuItem("Step In", "F10")) {
                //step debugger
                arm_handle->Step();
                arm_handle->RegisterDump(data.reg_dump_buffer);
                gba_handle->stack_dump(data.stack_dump_buffer, arm_handle->GetReg(SP));
                gba_handle->draw_bit_map();
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Debug"))
        {
            ImGui::Checkbox("Toggle Controls", &settings.enable_controls);
            ImGui::Separator();
            if(ImGui::Checkbox("Toggle Debug", &settings.enable_debug)) {
                settings.enable_reg_file = false;
                settings.enable_stack = false;
                settings.enable_break_points = false;
            }
            ImGui::Separator();
            ImGui::Checkbox("Toggle Register File", &settings.enable_reg_file);
            ImGui::Checkbox("Toggle Stack Viewer", &settings.enable_stack);
            ImGui::Checkbox("Toggle Instruction Viewer", &settings.enable_instructions);
            ImGui::Checkbox("Toggle Break Points", &settings.enable_break_points);
            ImGui::Checkbox("Toggle Memory Viewer", &settings.enable_memory);

            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }
}
