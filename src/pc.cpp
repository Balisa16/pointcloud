
#include <glad.h>
#include <GLFW/glfw3.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <window.hpp>

#include <texture.h>
#include <vao.h>
#include <vbo.h>
#include <ebo.h>
#include <camera.hpp>
#include <iomanip>

#include <fstream>
#include <sstream>

// #include <pointcloud.hpp>
#include <shader.hpp>
#include <reader.hpp>
#include <vao.h>
#include <ebo.h>
// Function to read the contents of a file and return it as a string

int width = 800, height = 800;

std::string shader_read(const char *filePath)
{
    std::ifstream file(filePath);
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

// Function to compile a shader
GLuint compileShader(GLenum shaderType, const char *shaderSource)
{
    GLuint shader = glCreateShader(shaderType);
    glShaderSource(shader, 1, &shaderSource, nullptr);
    glCompileShader(shader);

    // Check for compilation errors
    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        char infoLog[512];
        glGetShaderInfoLog(shader, sizeof(infoLog), nullptr, infoLog);
        std::cerr << "Shader compilation failed:\n"
                  << infoLog << std::endl;
    }

    return shader;
}

int main()
{
    PCDReader parser("../sample/pointcloud1.pcd");
    parser += "../sample/pointcloud3.pcd";
    PCDFormat data = parser.get_data();
    std::cout << data;

    Window win("Point Cloud", width, height);
    GLFWwindow *window = win.get_window();

    Shader shader("../script/pc.vert", "../script/pc.frag");

    VAO VAO1;
    VAO1.Bind();

    VBO VBO1(data.gl_data, data.num_points * 6 * sizeof(GLfloat));

    VAO1.LinkAttrib(VBO1, 0, 3, GL_FLOAT, 6 * sizeof(float), (void *)0);
    VAO1.LinkAttrib(VBO1, 1, 3, GL_FLOAT, 6 * sizeof(float), (void *)(3 * sizeof(float)));

    VAO1.Unbind();
    VBO1.Unbind();

    glEnable(GL_DEPTH_TEST);

    Camera cam(width, height, glm::vec3(0.0f, 0.0f, 2.0f));

    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        glClearColor(0.01f, 0.01f, 0.01f, 1.0f);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        shader.Activate();

        cam.Inputs(window);

        cam.Matrix(45.0f, 0.1f, 100.0f, shader.ID, "camera_view_mat");

        glDrawArrays(GL_POINTS, 0, data.num_points);

        VAO1.Bind();

        glfwSwapBuffers(window);
        // Take care of all GLFW events
        glfwPollEvents();
    }

    shader.Delete();
    VAO1.Delete();
    VBO1.Delete();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}