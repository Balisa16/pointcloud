#ifndef SHADER_CLASS_H
#define SHADER_CLASS_H

#include <glad.h>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cerrno>

std::string get_file_contents(const char *filename);

class Shader
{
public:
    GLuint ID;
    Shader(std::string vertex_file, std::string fragment_file);
    void Activate();
    void Delete();

private:
    std::string vertex_code, fragment_code;
    void read_content(std::string &target, std::string &file_path);
    void compileErrors(unsigned int shader, const char *type);
};

#endif