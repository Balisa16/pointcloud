#include <shader.hpp>

// Constructor that build the Shader Program from 2 different shaders
Shader::Shader(std::string vertex_file, std::string fragment_file)
{
    std::string initial_vert_path = vertex_file;
    std::string initial_frag_path = fragment_file;
    bool found_vert = false, found_frag = false;
    for (size_t i = 0; i < 3; i++)
    {
        if (boost::filesystem::exists(initial_vert_path))
        {
            found_vert = true;
            vertexfile = initial_vert_path;
            break;
        }
        initial_vert_path = "../" + initial_vert_path;
    }

    for (size_t i = 0; i < 3; i++)
    {
        if (boost::filesystem::exists(initial_frag_path))
        {
            found_frag = true;
            fragmentfile = initial_frag_path;
            break;
        }
        initial_frag_path = "../" + initial_frag_path;
    }

    if (!found_vert)
    {
        std::cout << "Can't find vertex file\n";
        exit(EXIT_FAILURE);
    }

    if (!found_frag)
    {
        std::cout << "Can't find fragment file\n";
        exit(EXIT_FAILURE);
    }

    read_content(vertex_code, vertexfile);
    read_content(fragment_code, fragmentfile);

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

    _id = glCreateProgram();
    glAttachShader(_id, _vertex_shader);
    glAttachShader(_id, _fragment_shader);
    glLinkProgram(_id);
    compileErrors(_id, "PROGRAM");

    // Delete the now useless Vertex and Fragment Shader objects
    glDeleteShader(_vertex_shader);
    glDeleteShader(_fragment_shader);
}

GLuint Shader::get_id()
{
    return _id;
}

void Shader::read_content(std::string &target, std::string &file_path)
{
    std::ifstream file(file_path);
    std::stringstream buffer;
    buffer << file.rdbuf();
    target = buffer.str();
}

// Activates the Shader Program
void Shader::activate()
{
    glUseProgram(_id);
}

// Deletes the Shader Program
void Shader::Delete()
{
    glDeleteProgram(_id);
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