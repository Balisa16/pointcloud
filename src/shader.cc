#include <shader.hpp>

// Constructor that build the Shader Program from 2 different shaders
Shader::Shader(std::string vertex_file, std::string fragment_file)
{
    read_content(vertex_code, vertex_file);
    read_content(fragment_code, fragment_file);

    const char *vcode = vertex_code.c_str();
    const char *fcode = fragment_code.c_str();

    GLuint _vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(_vertex_shader, 1, &vcode, NULL);
    glCompileShader(_vertex_shader);
    compileErrors(_vertex_shader, "VERTEX");

    GLuint _fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(_fragment_shader, 1, &fcode, NULL);
    glCompileShader(_fragment_shader);
    compileErrors(_fragment_shader, "FRAGMENT");

    ID = glCreateProgram();
    glAttachShader(ID, _vertex_shader);
    glAttachShader(ID, _fragment_shader);
    glLinkProgram(ID);
    compileErrors(ID, "PROGRAM");

    // Delete the now useless Vertex and Fragment Shader objects
    glDeleteShader(_vertex_shader);
    glDeleteShader(_fragment_shader);
}

void Shader::read_content(std::string &target, std::string &file_path)
{
    std::ifstream file(file_path);
    std::stringstream buffer;
    buffer << file.rdbuf();
    target = buffer.str();
}

// Activates the Shader Program
void Shader::Activate()
{
    glUseProgram(ID);
}

// Deletes the Shader Program
void Shader::Delete()
{
    glDeleteProgram(ID);
}

// Checks if the different Shaders have compiled properly
void Shader::compileErrors(unsigned int shader, const char *type)
{
    // Stores status of compilation
    GLint hasCompiled;
    // Character array to store error message in
    char infoLog[1024];
    if (type != "PROGRAM")
    {
        glGetShaderiv(shader, GL_COMPILE_STATUS, &hasCompiled);
        if (hasCompiled == GL_FALSE)
        {
            glGetShaderInfoLog(shader, 1024, NULL, infoLog);
            std::cout << "SHADER_COMPILATION_ERROR for:" << type << "\n"
                      << infoLog << std::endl;
        }
    }
    else
    {
        glGetProgramiv(shader, GL_LINK_STATUS, &hasCompiled);
        if (hasCompiled == GL_FALSE)
        {
            glGetProgramInfoLog(shader, 1024, NULL, infoLog);
            std::cout << "SHADER_LINKING_ERROR for:" << type << "\n"
                      << infoLog << std::endl;
        }
    }
}