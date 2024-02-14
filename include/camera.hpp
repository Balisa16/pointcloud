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
    // get instance
    static Camera &get()
    {
        static Camera camera;
        return camera;
    }

    static void init(GLFWwindow *window, int width, int height, glm::vec3 position)
    {
        get().camera_init(window, width, height, position);
    }

    static void Matrix(float FOVdeg, float nearPlane, float farPlane, GLuint shader_id, const char *uniform)
    {
        get().camera_matrix(FOVdeg, nearPlane, farPlane, shader_id, uniform);
    }

    static void Inputs(GLFWwindow *window)
    {
        get().camera_inputs(window);
    }

private:
    Camera(const Camera &) = delete;
    Camera &operator=(const Camera &) = delete;

    Camera() = default;
    ~Camera() = default;

    static void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);

    void camera_init(GLFWwindow *window, int width, int height, glm::vec3 position);

    void camera_matrix(float FOVdeg, float nearPlane, float farPlane, GLuint shader_id, const char *uniform);

    void camera_inputs(GLFWwindow *window);

    void mouse_scroll(bool up);

private:
    GLFWwindow *window;
    glm::vec3 Position;
    glm::vec3 Orientation = glm::vec3(0.0f, 0.0f, -1.0f);
    glm::vec3 Up = glm::vec3(0.0f, 1.0f, 0.0f);

    // Prevents the camera from jumping around when first clicking left click
    bool firstClick = true;
    int width;
    int height;
    float speed = 0.1f;
    float sensitivity = 100.0f;
};