#pragma once

#include <glad.h>
#include <GLFW/glfw3.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/vector_angle.hpp>

#include <shader.hpp>
class Camera
{
public:
    Camera(GLFWwindow *window, int width, int height, glm::vec3 position);

    ~Camera();

    void Matrix(float FOVdeg, float nearPlane, float farPlane, GLuint shader_id, const char *uniform);

    void Inputs(GLFWwindow *window);

    static void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);

    void handle_scroll(double yoffset);

private:
    glm::vec3 Position;
    glm::vec3 Orientation = glm::vec3(0.0f, 0.0f, -1.0f);
    glm::vec3 Up = glm::vec3(0.0f, 1.0f, 0.0f);

    // Prevents the camera from jumping around when first clicking left click
    bool firstClick = true;

    // Stores the width and height of the window
    int width;
    int height;

    // Adjust the speed of the camera and it's sensitivity when looking around
    float speed = 0.1f;
    float sensitivity = 100.0f;
};