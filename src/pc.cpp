
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

#include <texture.h>
#include <shader.h>
#include <vao.h>
#include <vbo.h>
#include <ebo.h>
#include <camera.h>
#include <iomanip>

#include <fstream>
#include <sstream>

#include <pointcloud.hpp>

// Function to read the contents of a file and return it as a string

const int width = 800, height = 800;

GLuint indices[] = {
    0, 1, 2,
    0, 2, 3,
    0, 1, 4,
    1, 2, 4,
    2, 3, 4,
    3, 0, 4};

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
    PCD pcd("../sample/pointcloud.pcd");
    pcl::PointCloud<pcl::PointXYZRGB> cloud = pcd.get();
    std::cout << cloud.points.size() << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    GLfloat points[cloud.points.size() * 6];
    for (int _i = 0; _i < cloud.points.size(); _i++)
    {
        points[_i * 6] = cloud.points[_i].x;
        points[_i * 6 + 1] = cloud.points[_i].y;
        points[_i * 6 + 2] = cloud.points[_i].z;
        points[_i * 6 + 3] = 1.0;
        points[_i * 6 + 4] = 1.0;
        points[_i * 6 + 5] = 1.0;
        std::cout << int(cloud.points[_i].r) << " " << int(cloud.points[_i].g) << " " << int(cloud.points[_i].b) << '\n';
    }

    glfwInit();

    // Tell GLFW what version of OpenGL we are using
    // In this case we are using OpenGL 3.3
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    // Tell GLFW we are using the CORE profile
    // So that means we only have the modern functions
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Create a GLFWwindow object of 800 by 800 pixels, naming it "YoutubeOpenGL"
    GLFWwindow *window = glfwCreateWindow(width, height, "OpenGL", NULL, NULL);
    // Error check if the window fails to create
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    // Introduce the window into the current context
    glfwMakeContextCurrent(window);

    gladLoadGL();
    // Specify the viewport of OpenGL in the Window
    // In this case the viewport goes from x = 0, y = 0, to x = 800, y = 800
    glViewport(0, 0, width, height);

    // Generates Shader object using shaders default.vert and default.frag
    // Shader shaderProgram("../script/pc.vert", "../script/pc.frag");

    // // Generates Vertex Array Object and binds it
    // VAO VAO1;
    // VAO1.Bind();

    // // Generates Vertex Buffer Object and links it to vertices
    // VBO VBO1(points, sizeof(points));
    // // Generates Element Buffer Object and links it to indices
    // EBO EBO1(indices, sizeof(indices));

    // // Links VBO attributes such as coordinates and colors to VAO
    // VAO1.LinkAttrib(VBO1, 0, 3, GL_FLOAT, 6 * sizeof(float), (void *)0);
    // // VAO1.LinkAttrib(VBO1, 1, 3, GL_FLOAT, 6 * sizeof(float), (void *)(3 * sizeof(float)));
    // // Unbind all to prevent accidentally modifying them
    // VAO1.Unbind();
    // VBO1.Unbind();
    // EBO1.Unbind();

    std::string vertexShaderSource = shader_read("../script/pc.vert");
    std::string fragmentShaderSource = shader_read("../script/pc.frag");

    // Compile shaders
    GLuint vertexShader = compileShader(GL_VERTEX_SHADER, vertexShaderSource.c_str());
    GLuint fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentShaderSource.c_str());

    // Link shaders
    GLuint shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    // Cleanup shaders
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    GLuint VAO, VBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(points), points, GL_STATIC_DRAW);

    // Set the attribute pointers
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid *)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    // Enables the Depth Buffer
    glEnable(GL_DEPTH_TEST);

    Camera cam(width, height, glm::vec3(0.0f, 0.0f, 10.0f));

    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        glClearColor(0.01f, 0.01f, 0.01f, 1.0f);
        // Clean the back buffer and depth buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glClear(GL_COLOR_BUFFER_BIT);

        // Use the shader program
        glUseProgram(shaderProgram);
        glBindVertexArray(VAO);

        // Set the modelViewProjection matrix (for simplicity, you can use an identity matrix here)
        glm::mat4 modelViewProjection = glm::mat4(1.0);
        GLuint mvpLocation = glGetUniformLocation(shaderProgram, "camera_view_mat");
        glUniformMatrix4fv(mvpLocation, 1, GL_FALSE, glm::value_ptr(modelViewProjection));

        // Draw the point cloud
        glDrawArrays(GL_POINTS, 0, sizeof(points) / (3 * sizeof(GLfloat)));

        glBindVertexArray(0);
        glUseProgram(0);

        glfwSwapBuffers(window);
        // Take care of all GLFW events
        glfwPollEvents();
    }

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteProgram(shaderProgram);
    // Delete window before ending the program
    glfwDestroyWindow(window);
    // Terminate GLFW before ending the program
    glfwTerminate();
    return 0;
}