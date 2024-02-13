#pragma once

#include <glad.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <GLFW/glfw3.h>
#include <iostream>
#include <string>

class Window
{
private:
    const char *title;
    int height, width;
    GLFWwindow *window;

private:
    static void resize_callback(GLFWwindow *window, int width, int height);

public:
    Window(std::string title = "Pointcloud", int width = 800, int height = 600);
    GLFWwindow *get_window();
    void window_resize(int width, int height);
    virtual ~Window();
};