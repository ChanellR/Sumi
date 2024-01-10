#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <ImGuiFileDialog.h>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <Application.hpp>
#include <GameWindow.hpp>
#include <utils.hpp>

const GLuint SCALE = 3;
const GLuint GAME_WINDOW_WIDTH = SCALE * 240, GAME_WINDOW_HEIGHT = SCALE * 160;

GameWindow::GameWindow(Emulator& emulator)
: emu(emulator), Window(std::string("Sumi"), GAME_WINDOW_WIDTH, GAME_WINDOW_HEIGHT)
{
    // Should still be the current context after window constructor

    glfwSetKeyCallback(handle_, [](GLFWwindow* window, int key, int scancode, int action, int mode) {
        // if (key == GLFW_KEY_ESCAPE) 
        //     glfwSetWindowShouldClose(window, GL_TRUE);
    });

    glfwSetFramebufferSizeCallback(handle_, [](GLFWwindow* window, int width, int height) {
        glViewport(0, 0, width, height);
    });

    glGenTextures(1, &bitmap_texture_);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, bitmap_texture_);
    
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    int image_width, image_height, image_channels;
    // stbi_set_flip_vertically_on_load(true); 
    unsigned char* data = stbi_load("../assets/pic.png", &image_width, &image_height, &image_channels, 0);

    if (data) 
    {
        GLFWimage icon {.width = image_width, .height = image_height, .pixels = data};
        glfwSetWindowIcon(handle_, 1, &icon);
        GL_CHECK(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image_width, image_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data));
    } 
    STBI_FREE(data);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); //fill the triangles as apposed to wireframe mode

    bitmap_shader_ = new ShaderProgram("../shaders/bitmap.vert", "../shaders/bitmap.frag");
    bitmap_VAO_ = new BitmapVertexArrayObject();

    //setup Imgui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    // ImGuiIO& io = ImGui::GetIO(); (void)io;
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    ImGui_ImplGlfw_InitForOpenGL(handle_, true);
    ImGui_ImplOpenGL3_Init("#version 330");

} 

GameWindow::~GameWindow() 
{
    glDeleteTextures(1, &bitmap_texture_);
    delete bitmap_shader_;
    delete bitmap_VAO_;

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(handle_);
}

void GameWindow::Update(float delta)
{
    glfwMakeContextCurrent(handle_);
    
    DispatchMessages(); 
    glfwPollEvents();

    UpdateImgui();
    
    if (glfwWindowShouldClose(handle_))
    {
        //Push destruction message
        return;
    }
    
    emu.Update();
    RenderFrame(); 

    glfwSwapBuffers(handle_);
}

void GameWindow::RenderFrame() 
{
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    //setup a texture for the bitmap to be pushed too
    //then render the texture to a quad that covers the screen

    bitmap_VAO_->Bind();
    bitmap_shader_->use();
    // //update the bitmap, pull from the emulator VRAM
    // glBindTexture(GL_TEXTURE_2D, bitmap_texture_);
    // emu.Display();
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    bitmap_VAO_->Unbind();

    //ImGui
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

}

void GameWindow::DispatchMessages() 
{

}

void GameWindow::PushMessage(Message message)
{

}

void GameWindow::UpdateImgui()
{
    // Since you switch context when updating
    // I assume that this will work regardless
    // of having seperate windows. 
    ImGui_ImplOpenGL3_NewFrame(); 
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("File"))
        {
            // All will send messages to the App
            if (ImGui::MenuItem("Load Game")) 
            {
                ImGuiFileDialog::Instance()->OpenDialog("ChooseFileDlgKey", "Choose File", ".elf,.3ds,.gba", "../");
            }

            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Emulation"))
        {
            if (ImGui::MenuItem("Pause/Play")) 
            {
                emu.Pause();
            }

            if (ImGui::MenuItem("Reboot")) 
            {
                emu.Reset();
            }

            if (ImGui::MenuItem("Step")) 
            {
                emu.Step();
            }
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }

    // display
    if (ImGuiFileDialog::Instance()->Display("ChooseFileDlgKey")) 
    {
        // action if OK
        if (ImGuiFileDialog::Instance()->IsOk())
        {
            std::string file_path = ImGuiFileDialog::Instance()->GetFilePathName();
            // std::string filePath = ImGuiFileDialog::Instance()->GetCurrentPath();
            std::cout << "Loading Rom: " << file_path << std::endl;
            emu.Reset();
            emu.LoadRom(file_path);
        }   
        
        // close
        ImGuiFileDialog::Instance()->Close();
    }

    ImGui::Render();
}