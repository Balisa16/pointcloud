#include <window.hpp>

Window::Window(std::string title, int width, int height)
    : title(title.c_str()), width(width), height(height)
{
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window = glfwCreateWindow(width, height, title.c_str(), NULL, NULL);

    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window\n";
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glfwMakeContextCurrent(window);

    gladLoadGL();

    glViewport(0, 0, width, height);

    glfwSetFramebufferSizeCallback(window, resize_callback);
}

GLFWwindow *Window::get_window()
{
    return window;
}

void Window::clear()
{
    glClearColor(bg_color(0), bg_color(1), bg_color(2), bg_color(3));
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Window::resize_callback(GLFWwindow *window, int width, int height)
{
    glViewport(0, 0, width, height);
}

template <>
void Window::set_bg_color(const Eigen::Vector4f color)
{
    bg_color = color;
}

template <>
void Window::set_bg_color(const Eigen::Vector3f color)
{
    bg_color = Eigen::Vector4f(color(0), color(1), color(2), 1.0f);
}

void Window::window_resize(int width, int height)
{
    this->width = width;
    this->height = height;
    glViewport(0, 0, width, height);
}

Window::~Window()
{
    glfwTerminate();
}