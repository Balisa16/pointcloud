#pragma once

#include <glad.h>
#include <stb_image.h>

#include <shader.hpp>

class Texture
{
public:
    Texture(const char *image, GLenum texType, GLenum slot, GLenum format, GLenum pixelType);

    // Assigns a texture unit to a texture
    void texUnit(Shader &shader, const char *uniform, GLuint unit);
    // Binds a texture
    void bind();

    GLuint get_id();

    GLenum get_type();
    // Unbinds a texture
    void unbind();
    // Deletes a texture
    void Delete();

private:
    GLuint _id;
    GLenum type;
};