#ifndef WINDOW_H
#define WINDOW_H

#include <vector>
#include <string>
#include <iostream>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#include <Application.hpp>
#include <Message.hpp>

class Application;

class Window
{
public:

    Window(std::string name, GLuint init_width, GLuint init_height)
    {   
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);

        handle_ = glfwCreateWindow(init_width, init_height, name.c_str(), NULL, NULL);
        glfwMakeContextCurrent(handle_); 
        if(!handle_) 
        {
            std::cout << "Failed to create GLFW Window: " << name << std::endl;
            return; 
        }

        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) 
        {
            std::cout << "Failed to initialize OpenGl context!" << std::endl;
            return; 
        }

        glViewport(0, 0, init_width, init_height);
    }

    virtual ~Window()
    {
        glfwDestroyWindow(handle_);
    }

    // Make window the current conext when updating
    // in individual update functions 
    virtual void Update(float delta) = 0;
    
    GLFWwindow* GetHandle() const {return handle_;}
    void SetApp(Application* root) {root_app_ = root;}
    std::vector<Message>& GetInbox() {return inbox_;}

protected:
    virtual void DispatchMessages() = 0;
    virtual void PushMessage(Message message) = 0;

    Application* root_app_;
    GLFWwindow* handle_;
    
    std::vector<Message> inbox_;
};

#endif