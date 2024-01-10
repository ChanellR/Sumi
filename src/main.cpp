#include <iostream>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>

#include "../include/ShaderProgram.hpp"
#include <Application.hpp>

int main (int argc, char* argv[]) 
{

    auto app = Application();
    while (app.IsRunning())
    {
        app.Tick();
    }

    return 0;
}