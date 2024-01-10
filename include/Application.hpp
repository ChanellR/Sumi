#ifndef APPLICATION_H
#define APPLICATION_H

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <memory>

#include <GameWindow.hpp>
#include <Message.hpp>

const GLuint WIDTH = 800, HEIGHT = 600;

class Application 
{
public:
    
    Application();
    ~Application();
    void Tick();
    bool IsRunning() const { return running;}
    void ReceiveMessage(Message message);

private:

    void DispatchMessages();
    std::vector<Message> inbox;
    bool running;
    GameWindow* game_window;
    Emulator* emulator;
};

#endif