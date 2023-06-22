#include "../include/glad/glad.h"
#include "../include/GLFW/glfw3.h"
#include "../imgui/imgui.h"
#include "../imgui/imgui_impl_glfw.h"
#include "../imgui/imgui_impl_opengl3.h"

#define STB_IMAGE_IMPLEMENTATION
#include "../include/extra/stb_image.h"

#include <iostream>
#include <map>
#include <windows.h>
#include "../include/Sumi/window.hpp"

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
bool test_key_press(GLFWwindow *window, uint16_t key, std::map<uint16_t, uint8_t> &last_presses);
bool LoadTextureFromFile(const char* filename, GLuint* out_texture, int* out_width, int* out_height);
void open_memory_table(GBA* gba_handle, uint32_t base_address);

// settings
const unsigned int SCR_WIDTH = 1200;
const unsigned int SCR_HEIGHT = 800;

int run_app(Arm* arm_handle, GBA* gba_handle)
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
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

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


    // render loop
    // -----------

    char reg_dump_buffer[16 * REGSIZE];
    char stack_dump_buffer[24*15];
    bool show_demo = true;
    int search_address = 0x05000000;
    std::map<uint16_t, uint8_t> key_presses;
    arm_handle->register_dump(reg_dump_buffer);
    gba_handle->stack_dump(stack_dump_buffer, arm_handle->rf[SP]);


    GLuint renderedTexture;
    glGenTextures(1, &renderedTexture);
    glBindTexture(GL_TEXTURE_2D, renderedTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 240, 160, 0,
        GL_RGB, GL_UNSIGNED_BYTE, gba_handle->FRAME);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    // glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, renderedTexture, 0);

    // int my_image_width = 0;
    // int my_image_height = 0;
    // GLuint my_image_texture = 0;
    // bool ret = LoadTextureFromFile("frisk.png", &my_image_texture, &my_image_width, &my_image_height);
    // IM_ASSERT(ret);


    while (!glfwWindowShouldClose(window))
    {
        ImGui_ImplGlfw_NewFrame();
        ImGui_ImplOpenGL3_NewFrame();
        ImGui::NewFrame();

        // double start = ImGui::GetTime();
        // while(ImGui::GetTime() - start < (1000/60.0f)) arm_handle->step();

        
        //Dear Imgui
        {
 
            ImGui::Begin("Register Dump");
            ImGui::Text(reg_dump_buffer);
            ImGui::End();

            ImGui::Begin("Instruction Memory");
            ImGui::BeginChild("Scrolling");
            
            {
                uint32_t pc = arm_handle->get_pc();
                for (int word = -10; word < 10; word++){
                    if((int32_t)(pc + (word*4)) < 0) continue;

                    char mnemonic[35];
                    Arm::MemOp fetch_operation {pc + (word*4), Arm::ldw};
                    uint32_t instruction = arm_handle->MMU(fetch_operation);

                    arm_handle->disassemble(mnemonic, instruction, (pc + (word*4))); 

                    if (word == 0) {
                        ImGui::TextColored(ImVec4(0,1,0.6,0.9),"0x%08X: 0x%08X  %s", pc + (4*word), instruction, mnemonic);
                    } else {
                        ImGui::Text("0x%08X: 0x%08X  %s", pc + (4*word), instruction, mnemonic);
                    }
                }
            }

            ImGui::EndChild();
            ImGui::End();

            ImGui::Begin("Stack");
            ImGui::Text(stack_dump_buffer);
            ImGui::End();
            
            ImGui::Begin("BitMap Render");
            ImGui::Image((void*)(intptr_t)renderedTexture, ImVec2(240, 160));
            ImGui::End();

            ImGui::Begin("Memory Viewer");
            ImGuiInputTextFlags input_text_flags = ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_CharsHexadecimal | ImGuiInputTextFlags_EscapeClearsAll ;
            ImGui::InputInt("Search", &search_address, 1, 144, input_text_flags);
            open_memory_table(gba_handle, search_address);
            ImGui::End();

        }

        // input
        // -----
        glfwPollEvents();
        if(test_key_press(window, GLFW_KEY_S, key_presses)) {
            arm_handle->step();
            arm_handle->register_dump(reg_dump_buffer);
            gba_handle->stack_dump(stack_dump_buffer, arm_handle->rf[SP]);
            gba_handle->draw_bit_map();
        }
        if(test_key_press(window, GLFW_KEY_R, key_presses)) {
            arm_handle->reset(); 
            gba_handle->reset();
            arm_handle->register_dump(reg_dump_buffer);
            gba_handle->stack_dump(stack_dump_buffer, arm_handle->rf[SP]);
            gba_handle->draw_bit_map();
        }
        if(test_key_press(window, GLFW_KEY_ESCAPE, key_presses)) glfwSetWindowShouldClose(window, true);
        // render
        // ------
        glClearColor(0xC4/255.0f, 0xAA/255.0f, 0x7D/255.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
        Sleep((1000/60.0f));
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
bool test_key_press(GLFWwindow *window, uint16_t key, std::map<uint16_t, uint8_t> &last_presses){
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
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}



void open_memory_table(GBA* gba_handle, uint32_t base_address){

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
            Arm::MemOp op {row, Arm::ldw};
            ImGui::Text("0x%08X", gba_handle->memory_access(op));
        }
    }
    ImGui::EndTable();

}


// Simple helper function to load an image into a OpenGL texture with common settings
bool LoadTextureFromFile(const char* filename, GLuint* out_texture, int* out_width, int* out_height)
{
    // Load from file
    int image_width = 0;
    int image_height = 0;
    unsigned char* image_data = stbi_load(filename, &image_width, &image_height, NULL, 4);
    if (image_data == NULL)
        return false;

    // Create a OpenGL texture identifier
    GLuint image_texture;
    glGenTextures(1, &image_texture);
    glBindTexture(GL_TEXTURE_2D, image_texture);

    // Setup filtering parameters for display
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE); // This is required on WebGL for non power-of-two textures
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE); // Same

    // Upload pixels into texture
#if defined(GL_UNPACK_ROW_LENGTH) && !defined(__EMSCRIPTEN__)
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
#endif
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image_width, image_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data);
    stbi_image_free(image_data);

    *out_texture = image_texture;
    *out_width = image_width;
    *out_height = image_height;

    return true;
}