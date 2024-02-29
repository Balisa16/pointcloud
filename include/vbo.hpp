#ifndef VBO_CLASS_H
#define VBO_CLASS_H

#include <glad.h>
#include <iostream>

class VBO
{
public:
    // Reference ID of the Vertex Buffer Object
    // Constructor that generates a Vertex Buffer Object and links it to vertices
    VBO(GLfloat *vertices, GLsizeiptr size);

    void bind();

    GLuint get_id();

    void set_vertices(uint32_t data1_idx,
                      uint32_t data2_idx,
                      uint32_t data1_len,
                      uint32_t data2_len);

    void unbind();

    virtual ~VBO();

private:
    GLuint _id;
};

#endif