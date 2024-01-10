#include <iostream>
#include <Application.hpp>
#include <GameWindow.hpp>

Application::Application() : running(true)
{
    glfwInit();
    emulator = new Emulator();
    game_window = new GameWindow(*emulator);
    game_window->SetApp(this);
}

Application::~Application()
{
    delete game_window;
    delete emulator;
    glfwTerminate();
}

// For everything, the debug window and emulator
// should they run on different threads?
// or we can just call there individual functions later
// I'll probably split the windows up after this
void Application::Tick() 
{
    if(glfwWindowShouldClose(game_window->GetHandle()))
        running = false;

    
    game_window->Update(0);
}

void Application::ReceiveMessage(Message message)
{
    inbox.push_back(message);
}

void Application::DispatchMessages()
{
    
}