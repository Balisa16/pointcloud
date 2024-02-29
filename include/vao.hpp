#ifndef VAO_CLASS_H
#define VAO_CLASS_H

#include <glad.h>
#include <vbo.hpp>

class VAO
{
public:
    VAO();

    void bind();

    GLuint get_id();

    void unbind();

    virtual ~VAO();

private:
    GLuint _id;
};

#endif