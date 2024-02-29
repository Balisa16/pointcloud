#include <vbo.hpp>

// Constructor that generates a Vertex Buffer Object and links it to vertices
VBO::VBO(GLfloat *vertices, GLsizeiptr size)
{
    glGenBuffers(1, &_id);
    glBindBuffer(GL_ARRAY_BUFFER, _id);
    glBufferData(GL_ARRAY_BUFFER, size, vertices, GL_STATIC_DRAW);
}

// Binds the VBO
void VBO::bind()
{
    glBindBuffer(GL_ARRAY_BUFFER, _id);
}

GLuint VBO::get_id()
{
    return _id;
}

void VBO::set_vertices(uint32_t data1_idx, uint32_t data2_idx, uint32_t data1_len, uint32_t data2_len)
{
    glVertexAttribPointer(data1_idx, data1_len, GL_FLOAT, GL_FALSE, (data1_len + data2_len) * sizeof(GLfloat), (void *)0);
    glEnableVertexAttribArray(data1_idx);
    glVertexAttribPointer(data2_idx, data2_len, GL_FLOAT, GL_FALSE, (data1_len + data2_len) * sizeof(GLfloat), (void *)(data1_len * sizeof(GLfloat)));
    glEnableVertexAttribArray(data2_idx);
}

void VBO::unbind()
{
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

VBO::~VBO()
{
    glDeleteBuffers(1, &_id);
}