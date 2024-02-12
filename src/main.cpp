// #include <iostream>
// // #include <GL/glew.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <glad.h>
// #include <GLFW/glfw3.h>

// // Function to draw the point cloud
// void drawPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
// {
//     glPointSize(2.0f);
//     glBegin(GL_POINTS);

//     for (const auto &point : cloud->points)
//     {
//         if (!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z))
//         {
//             glColor3ub(255, 0, 0);
//             glVertex3f(point.x, point.y, point.z);
//         }
//     }

//     glEnd();
// }

// GLFWwindow *initWindow(int width, int height, const char *title)
// {
//     if (!glfwInit())
//     {
//         std::cerr << "Failed to initialize GLFW" << std::endl;
//         exit(EXIT_FAILURE);
//     }

//     glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
//     glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
//     glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

//     // Disable resize
//     glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

//     GLFWwindow *window = glfwCreateWindow(width, height, title, nullptr, nullptr);
//     if (!window)
//     {
//         std::cerr << "Failed to create GLFW window" << std::endl;
//         glfwTerminate();
//         exit(EXIT_FAILURE);
//     }

//     glfwMakeContextCurrent(window);

//     gladLoadGL();

//     glEnable(GL_DEPTH_TEST);
//     glViewport(0, 0, width, height);

//     glClearColor(0.02f, 0.02f, 0.02f, 1.0f);
//     glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//     glfwSwapBuffers(window);

//     return window;
// }

// int main()
// {
//     // Read PCD file
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     if (pcl::io::loadPCDFile<pcl::PointXYZ>("../sample/ICRA_willow_challenge/T_02/cloud_0000000000.pcd", *cloud) == -1)
//     {
//         std::cerr << "Couldn't read PCD file" << std::endl;
//         return -1;
//     }

//     GLFWwindow *window = initWindow(640, 480, "Point Cloud Viewer");
//     if (!window)
//     {
//         std::cerr << "Failed to create GLFW window" << std::endl;
//         glfwTerminate();
//         return -1;
//     }

//     while (!glfwWindowShouldClose(window))
//     {
//         glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//         // Draw point cloud
//         drawPointCloud(cloud);

//         glfwSwapBuffers(window);
//         glfwPollEvents();
//     }

//     // Cleanup
//     glfwDestroyWindow(window);
//     glfwTerminate();

//     return 0;
// }

//------- Ignore this ----------
#include <filesystem>
namespace fs = std::filesystem;
//------------------------------

#include <iostream>
#include <glad.h>
#include <GLFW/glfw3.h>
#include <stb_image.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <texture.h>
#include <shader.h>
#include <vao.h>
#include <vbo.h>
#include <ebo.h>
#include <camera.hpp>

const unsigned int width = 800;
const unsigned int height = 800;

// Vertices coordinates
GLfloat vertices[] =
    { //     COORDINATES     /        COLORS      /   TexCoord  //
        -0.5f, 0.0f, 0.5f, 0.83f, 0.70f, 0.44f, 0.0f, 0.0f,
        -0.5f, 0.0f, -0.5f, 0.83f, 0.70f, 0.44f, 5.0f, 0.0f,
        0.5f, 0.0f, -0.5f, 0.83f, 0.70f, 0.44f, 0.0f, 0.0f,
        0.5f, 0.0f, 0.5f, 0.83f, 0.70f, 0.44f, 5.0f, 0.0f,
        0.0f, 0.8f, 0.0f, 0.92f, 0.86f, 0.76f, 2.5f, 5.0f};

// Indices for vertices order
GLuint indices[] =
    {
        0, 1, 2,
        0, 2, 3,
        0, 1, 4,
        1, 2, 4,
        2, 3, 4,
        3, 0, 4};

int main()
{
    // Initialize GLFW
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

    // Load GLAD so it configures OpenGL
    gladLoadGL();
    // Specify the viewport of OpenGL in the Window
    // In this case the viewport goes from x = 0, y = 0, to x = 800, y = 800
    glViewport(0, 0, width, height);

    // Generates Shader object using shaders default.vert and default.frag
    Shader shaderProgram("../script/default.vert", "../script/default.frag");

    // Generates Vertex Array Object and binds it
    VAO VAO1;
    VAO1.Bind();

    // Generates Vertex Buffer Object and links it to vertices
    VBO VBO1(vertices, sizeof(vertices));
    // Generates Element Buffer Object and links it to indices
    EBO EBO1(indices, sizeof(indices));

    // Links VBO attributes such as coordinates and colors to VAO
    VAO1.LinkAttrib(VBO1, 0, 3, GL_FLOAT, 8 * sizeof(float), (void *)0);
    VAO1.LinkAttrib(VBO1, 1, 3, GL_FLOAT, 8 * sizeof(float), (void *)(3 * sizeof(float)));
    VAO1.LinkAttrib(VBO1, 2, 2, GL_FLOAT, 8 * sizeof(float), (void *)(6 * sizeof(float)));
    // Unbind all to prevent accidentally modifying them
    VAO1.Unbind();
    VBO1.Unbind();
    EBO1.Unbind();

    /*
     * I'm doing this relative path thing in order to centralize all the resources into one folder and not
     * duplicate them between tutorial folders. You can just copy paste the resources from the 'Resources'
     * folder and then give a relative path from this folder to whatever resource you want to get to.
     * Also note that this requires C++17, so go to Project Properties, C/C++, Language, and select C++17
     */
    std::string parentDir = (fs::current_path().fs::path::parent_path()).string();
    std::string texPath = "/assets/images/";

    // Texture
    Texture brickTex((parentDir + texPath + "brick.png").c_str(), GL_TEXTURE_2D, GL_TEXTURE0, GL_RGBA, GL_UNSIGNED_BYTE);
    brickTex.texUnit(shaderProgram, "tex0", 0);

    // Enables the Depth Buffer
    glEnable(GL_DEPTH_TEST);

    Camera cam(width, height, glm::vec3(0.0f, 0.0f, 2.0f));

    // Main while loop
    while (!glfwWindowShouldClose(window))
    {
        // Specify the color of the background
        glClearColor(0.07f, 0.13f, 0.17f, 1.0f);
        // Clean the back buffer and depth buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        // Tell OpenGL which Shader Program we want to use
        shaderProgram.Activate();

        cam.Inputs(window);

        cam.Matrix(45.0f, 0.1f, 100.0f, shaderProgram.ID, "camMatrix");

        // Binds texture so that is appears in rendering
        brickTex.Bind();
        // Bind the VAO so OpenGL knows to use it
        VAO1.Bind();
        // Draw primitives, number of indices, datatype of indices, index of indices
        glDrawElements(GL_TRIANGLES, sizeof(indices) / sizeof(int), GL_UNSIGNED_INT, 0);
        // Swap the back buffer with the front buffer
        glfwSwapBuffers(window);
        // Take care of all GLFW events
        glfwPollEvents();
    }

    // Delete all the objects we've created
    VAO1.Delete();
    VBO1.Delete();
    EBO1.Delete();
    brickTex.Delete();
    shaderProgram.Delete();
    // Delete window before ending the program
    glfwDestroyWindow(window);
    // Terminate GLFW before ending the program
    glfwTerminate();
    return 0;
}