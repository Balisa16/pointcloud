#pragma once

#include <glad.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <GLFW/glfw3.h>
#include <Eigen/Eigen>
#include <iostream>
#include <string>

class Window
{
private:
    const char *title;
    int height, width;
    GLFWwindow *window;
    Eigen::Vector4f bg_color = Eigen::Vector4f(0.1f, 0.1f, 0.1f, 1.0f);

private:
    static void resize_callback(GLFWwindow *window, int width, int height);

public:
    Window(std::string title = "Pointcloud", int width = 800, int height = 600);

    /**
     * @brief Get the window instance
     *
     * @return GLFWwindow* window
     */
    GLFWwindow *get_window();

    /**
     * @brief Clear the window
     *
     */
    void clear() const;

    /**
     * @brief Set the background color object
     *
     * @tparam T
     * @param color color code
     */
    template <typename T>
    void set_bg_color(const T color);

    /**
     * @brief Resize the window
     *
     * @param width width (px)
     * @param height height (px)
     */
    void window_resize(int width, int height);
    virtual ~Window();
};