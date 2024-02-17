#include <window.hpp>

Window::Window(std::string title, int width, int height)
    : title(title.c_str()), width(width), height(height)
{
    this->title = title.c_str();
    this->width = width;
    this->height = height;

    if (!glfwInit())
    {
        std::cout << "Failed to initialize GLFW\n";
        exit(EXIT_FAILURE);
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window = glfwCreateWindow(this->width, this->height, this->title, nullptr, nullptr);

    if (!window)
    {
        std::cout << "Failed to create GLFW window\n";
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glfwMakeContextCurrent(window);

    gladLoadGL();

    glViewport(0, 0, this->width, this->height);

    glfwSetFramebufferSizeCallback(window, resize_callback);
}

GLFWwindow *Window::get_window()
{
    return window;
}

void Window::clear() const
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
    glViewport(0, 0, this->width, this->height);
}

Window::~Window() {}