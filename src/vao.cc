#include <vao.hpp>

VAO::VAO()
{
    glGenVertexArrays(1, &_id);
}

void VAO::bind()
{
    glBindVertexArray(_id);
}

GLuint VAO::get_id()
{
    return _id;
}

void VAO::unbind()
{
    glBindVertexArray(0);
}

VAO::~VAO()
{
    glDeleteVertexArrays(1, &_id);
}