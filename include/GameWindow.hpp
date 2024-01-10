#ifndef GAME_WINDOW_H
#define GAME_WINDOW_H

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>

#include <ShaderProgram.hpp>
#include <Application.hpp>
#include <Window.hpp>
#include <Emulator.hpp>

enum RenderMode {Bitmap};

class Application;
class BitmapVertexArrayObject;

class GameWindow : public Window
{
public:
    GameWindow(Emulator& emulator);
    ~GameWindow();
    void Update(float delta) override;

private:
    void DispatchMessages() override;
    void PushMessage(Message message) override;
    void RenderFrame();
    void UpdateImgui();

    RenderMode mode_ = Bitmap;

    Emulator& emu;
    ShaderProgram* bitmap_shader_;
    BitmapVertexArrayObject* bitmap_VAO_;

    GLuint bitmap_texture_ = 0;
};

class BitmapVertexArrayObject 
{
public:
    BitmapVertexArrayObject()
    {
        glGenVertexArrays(1, &ID_); //vertex array object holds all the buffer, shader attribute information, elements, etc. 
        glBindVertexArray(ID_);

        float vertices[] = {
            // positions       //texture coords
            -1.0f, -1.0f, 0.0f, 0.0f, 0.0f, //bottom left  
            1.0f, -1.0f, 0.0f,  1.0f, 0.0f, //bottom right
            1.0f, 1.0f, 0.0f,   1.0f, 1.0f, //top right
            -1.0f, 1.0f, 0.0f,  0.0f, 1.0f, //top left
        };

        unsigned int indices[] = {
            0, 1, 2,
            2, 3, 0
        };

        // GLuint EBO;
        glGenBuffers(1, &EBO_);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

        // GLuint VBO;
        glGenBuffers(1, &VBO_); //create vertex buffer object
        glBindBuffer(GL_ARRAY_BUFFER, VBO_); //bind the buffer
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW); //fill buffer with vertices

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0); //configure positions
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));

        glEnableVertexAttribArray(0); //enable positions
        glEnableVertexAttribArray(1);
        
        glBindVertexArray(0); //unbind VAO
        glBindBuffer(GL_ARRAY_BUFFER, 0); //unbind VBO
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0); //unbind EBO
    }

    ~BitmapVertexArrayObject()
    {
        glDeleteVertexArrays(1, &ID_);
        glDeleteBuffers(1, &VBO_);
        glDeleteBuffers(1, &EBO_);
    }

    void Bind() 
    {
        glBindVertexArray(ID_);
    }

    void Unbind()
    {
        glBindVertexArray(0);
    }

private:

    GLuint ID_ = 0;
    GLuint VBO_ = 0;
    GLuint EBO_ = 0;

};

#endif