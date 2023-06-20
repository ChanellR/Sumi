#include "../include/glad/glad.h"
#include "../include/GLFW/glfw3.h"
#include "../imgui/imgui.h"
#include "../imgui/imgui_impl_glfw.h"
#include "../imgui/imgui_impl_opengl3.h"

#include <iostream>
#include <map>
#include "../include/Sumi/window.hpp"

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
bool test_key_press(GLFWwindow *window, uint16_t key, std::map<uint16_t, uint8_t> &last_presses);

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
    std::map<uint16_t, uint8_t> key_presses;
    key_presses[GLFW_KEY_ENTER] = GLFW_RELEASE;

    while (!glfwWindowShouldClose(window))
    {
        ImGui_ImplGlfw_NewFrame();
        ImGui_ImplOpenGL3_NewFrame();
        ImGui::NewFrame();
        
        //Dear Imgui
        {
            ImGui::Begin("Register Dump");
            arm_handle->dump(reg_dump_buffer);
            ImGui::Text(reg_dump_buffer);
            ImGui::End();

            ImGui::Begin("Instruction Memory");
            ImGui::BeginChild("Scrolling");
            
            {
                uint32_t pc = arm_handle->get_pc();
                for (int n = -10; n < 10; n++){
                    if((int32_t)(pc + (n*4)) < 0) continue;

                    MemOp fetch_operation = {pc + (n*4), ldd};
                    uint32_t instruction = gba_handle->memory_access(fetch_operation);

                    if (n == 0) {
                        ImGui::TextColored(ImVec4(0,1,0.5,0.4),"0x%08X: 0x%08X", pc + (4*n), instruction);
                    } else {
                        ImGui::Text("0x%08X: 0x%08X", pc + (4*n), instruction);
                    }
                }
            }

            ImGui::EndChild();
            ImGui::End();
        }

        // input
        // -----
        glfwPollEvents();
        if(test_key_press(window, GLFW_KEY_ENTER, key_presses)) arm_handle->step();
        if(test_key_press(window, GLFW_KEY_BACKSPACE, key_presses)) arm_handle->reset();

        // render
        // ------
        glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
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