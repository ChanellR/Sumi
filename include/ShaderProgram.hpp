#ifndef SHADER_PROGRAM_H
#define SHADER_PROGRAM_H

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

#include <glad/glad.h>

class ShaderProgram 
{
public:
    
    GLuint ID; 

    ShaderProgram(const char* vertex_path, const char* frag_path) 
    {
        std::string vertex_code_string, frag_code_string;
        std::ifstream vertex_file;
        std::ifstream frag_file;
        vertex_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        frag_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);

        try 
        {
            vertex_file.open(vertex_path);
            frag_file.open(frag_path);
            std::stringstream vertex_stream, frag_stream;
            //read the contents
            vertex_stream << vertex_file.rdbuf();
            frag_stream << frag_file.rdbuf();
            //close them 
            vertex_file.close();
            frag_file.close();
            vertex_code_string = vertex_stream.str();
            frag_code_string = frag_stream.str();
        }
        catch(std::ifstream::failure &e)
        {
            std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ" << std::endl;
        }

        const char* vertex_shader_source = vertex_code_string.c_str();
        const char* fragment_shader_source = frag_code_string.c_str();

        int success;
        char infoLog[512];

        GLuint vertexShader;
        vertexShader = glCreateShader(GL_VERTEX_SHADER);

        glShaderSource(vertexShader, 1, &vertex_shader_source, NULL);
        glCompileShader(vertexShader);

        glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);

        if(!success)
        {
            glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
            std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
        }

        GLuint fragmentShader;
        fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

        glShaderSource(fragmentShader, 1, &fragment_shader_source, NULL);
        glCompileShader(fragmentShader);

        glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);

        if(!success)
        {
            glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
            std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
        }

        GLuint shaderProgram;
        shaderProgram = glCreateProgram();
        ID = shaderProgram;

        glAttachShader(shaderProgram, vertexShader);
        glAttachShader(shaderProgram, fragmentShader);

        glLinkProgram(shaderProgram);

        glGetProgramiv(ID, GL_LINK_STATUS, &success);
        if (!success)
        {
            glGetProgramInfoLog(ID, 512, NULL, infoLog);
            std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
        }

        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);

    }

    ~ShaderProgram()
    {
        glDeleteProgram(ID);
    }
    
    void use()
    {
        glUseProgram(ID);
    }

    
};

#endif