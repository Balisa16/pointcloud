
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

// Vertex Shader
const char *vertexShaderSource = R"(
    #version 330 core
    layout (location = 0) in vec3 position;
    uniform mat4 modelViewProjection;
    void main() {
        gl_Position = modelViewProjection * vec4(position, 1.0);
        gl_PointSize = 5.0; // Adjust the point size as needed
    }
)";

// Fragment Shader
const char *fragmentShaderSource = R"(
    #version 330 core
    out vec4 fragColor;
    void main() {
        fragColor = vec4(1.0, 1.0, 1.0, 1.0); // White color for the point
    }
)";

const int width = 800, height = 800;

int main()
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../sample/ICRA_willow_challenge/T_02/cloud_0000000000.pcd", *cloud) == -1)
    {
        std::cerr << "Couldn't read PCD file" << std::endl;
        exit(EXIT_FAILURE);
    }

    GLfloat points[cloud->points.size() * 3];
    for (int i = 0; i < cloud->points.size(); i++)
    {
        points[i * 3] = cloud->points[i].x;
        points[i * 3 + 1] = cloud->points[i].y;
        points[i * 3 + 2] = cloud->points[i].z;
    }

    std::cout << cloud->points.size() << std::endl;

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

    // Load GLAD s

    // Compile shaders
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, nullptr);
    glCompileShader(vertexShader);

    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, nullptr);
    glCompileShader(fragmentShader);

    // Link shaders
    GLuint shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    // Cleanup shaders
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    // Create Vertex Array Object and Vertex Buffer Object
    GLuint VAO, VBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(points), points, GL_STATIC_DRAW);

    // Set the attribute pointers
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid *)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        glClear(GL_COLOR_BUFFER_BIT);

        // Use the shader program
        glUseProgram(shaderProgram);
        glBindVertexArray(VAO);

        // Set the modelViewProjection matrix (for simplicity, you can use an identity matrix here)
        glm::mat4 modelViewProjection = glm::mat4(1.0);
        GLuint mvpLocation = glGetUniformLocation(shaderProgram, "modelViewProjection");
        glUniformMatrix4fv(mvpLocation, 1, GL_FALSE, &modelViewProjection[0][0]);

        // Draw the point cloud
        glDrawArrays(GL_POINTS, 0, sizeof(points) / (3 * sizeof(GLfloat)));

        glBindVertexArray(0);
        glUseProgram(0);

        // Swap buffers and poll for events
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Cleanup
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteProgram(shaderProgram);

    glfwTerminate();
    return 0;
}