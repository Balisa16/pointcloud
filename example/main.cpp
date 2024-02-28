
#include <glad.h>
#include <GLFW/glfw3.h>
#include <GL/glut.h>
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
#include <vao.hpp>
#include <vbo.hpp>
#include <ebo.hpp>
#include <camera.hpp>
#include <iomanip>

#include <fstream>
#include <sstream>

// #include <pointcloud.hpp>
#include <shader.hpp>
#include <reader.hpp>
#include <filehandler.hpp>

int width = 800, height = 800;

int main()
{
    PCDReader parser("../../sample/pointcloud1.pcd");
    Buffer buff;
    // parser += "../../sample/pointcloud3.pcd";
    PCDFormat data = parser.get_data();
    buff = data;

    // PCDFormat data = parser.get_data();

    Window win("Point Cloud", width, height);
    GLFWwindow *window = win.get_window();

    Shader shader("../../script/pc.vert", "../../script/pc.frag");

    GLfloat line_data[96] = {
        0.f, 0.f, .0f, 1.f, .0f, .0f, .15f, .15f, .20f, 1.f, .0f, .0f,
        0.f, 0.f, .0f, 1.f, .0f, .0f, .15f, -.15f, .20f, 1.f, .0f, .0f,
        0.f, 0.f, .0f, 1.f, .0f, .0f, -.15f, .15f, .20, 1.f, .0f, .0f,
        0.f, 0.f, .0f, 1.f, .0f, .0f, -.15f, -.15f, .20f, 1.f, .0f, .0f,
        -.15f, -.15f, .20f, 1.f, .0f, .0f, .15f, -.15f, .20f, 1.f, .0f, .0f,
        .15f, -.15f, .20f, 1.f, .0f, .0f, .15f, .15f, .20f, 1.f, .0f, .0f,
        .15f, .15f, .20f, 1.f, .0f, .0f, -.15f, .15f, .20, 1.f, .0f, .0f,
        -.15f, .15f, .20, 1.f, .0f, .0f, -.15f, -.15f, .20f, 1.f, .0f, .0f};

    GLuint pointCloudVBO, lineVBO;
    GLuint pointCloudVAO, lineVAO;

    // Create and bind VAO
    glGenVertexArrays(1, &pointCloudVAO);
    glBindVertexArray(pointCloudVAO);

    // Create and bind VBO
    glGenBuffers(1, &pointCloudVBO);
    glBindBuffer(GL_ARRAY_BUFFER, pointCloudVBO);

    // Allocate memory for the buffer, and set the data (replace nullptr with your actual point cloud data)
    glBufferData(GL_ARRAY_BUFFER, data.num_points * 6 * sizeof(GLfloat), data.gl_data, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (void *)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (void *)(3 * sizeof(GLfloat)));
    glEnableVertexAttribArray(1);

    // Unbind VAO and VBO
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    // Create and bind VAO
    glGenVertexArrays(1, &lineVAO);
    glBindVertexArray(lineVAO);

    // Create and bind VBO
    glGenBuffers(1, &lineVBO);
    glBindBuffer(GL_ARRAY_BUFFER, lineVBO);

    // Allocate memory for the buffer, and set the data (replace nullptr with your actual line data)
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * 96, line_data, GL_STATIC_DRAW);

    // Set the vertex attribute pointers
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);

    // Unbind VAO and VBO
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    // GLuint vao, vbo;
    // glGenVertexArrays(1, &vao);
    // glBindVertexArray(vao);

    // glGenBuffers(1, &vbo);
    // glBindBuffer(GL_ARRAY_BUFFER, vbo);

    // glBufferData(GL_ARRAY_BUFFER, data.num_points * 6 * sizeof(GLfloat), data.gl_data, GL_STATIC_DRAW);

    // glBindBuffer(GL_ARRAY_BUFFER, vbo);
    // glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (void *)0);
    // glEnableVertexAttribArray(0);
    // glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (void *)(3 * sizeof(GLfloat)));
    // glEnableVertexAttribArray(1);
    // glBindVertexArray(0);
    // glBindBuffer(GL_ARRAY_BUFFER, 0);

    // VAO VAO1;
    // VAO1.Bind();

    // VBO VBO1(data.gl_data, data.num_points * 6 * sizeof(GLfloat));

    // VAO1.LinkAttrib(VBO1, 0, 3, GL_FLOAT, 6 * sizeof(float), (void *)0);
    // VAO1.LinkAttrib(VBO1, 1, 3, GL_FLOAT, 6 * sizeof(float), (void *)(3 * sizeof(float)));

    // VAO1.Unbind();
    // VBO1.Unbind();

    glEnable(GL_DEPTH_TEST);

    Camera::init(window, width, height, glm::vec3(0.0f, 0.0f, 0.0f));

    // Main loop

    // int counter = 3;

    // // std::promise<int> promise;
    // std::future<int> result_task;
    // bool is_run_task = false;

    FileHandler::read("../../sample");

    while (!glfwWindowShouldClose(window))
    {
        win.clear();

        shader.Activate();

        Camera::Inputs(window);

        Camera::Matrix(45.0f, 0.1f, 100.0f, shader.ID, "camera_view_mat");

        FileHandler::sync();

        // Bind Pointcloud
        glBindVertexArray(pointCloudVAO);
        =
            if (FileHandler::is_new_data())
        {
            Buffer _new_data;
            FileHandler::get_data(_new_data);

            buff += _new_data;
            glBindBuffer(GL_ARRAY_BUFFER, pointCloudVBO);
            glBufferData(GL_ARRAY_BUFFER, (buff.size() * 6 + buff.start()) * sizeof(GLfloat), buff.data, GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
        }

        // glDrawArrays(GL_LINES, 0, int(buff.start() / 6));
        glDrawArrays(GL_POINTS, buff.start() / 6 - 1, buff.size());
        glBindVertexArray(0);

        // Bind Camera Frame
        glBindVertexArray(lineVAO);
        glDrawArrays(GL_LINES, 0, numPoints);
        glBindVertexArray(0);

        glfwSwapBuffers(window);

        glfwPollEvents();
    }
    shader.Delete();
    // VAO1.Delete();
    // VBO1.Delete();

    glDeleteVertexArrays(1, &lineVAO);
    glDeleteVertexArrays(1, &lineVBO);

    glDeleteVertexArrays(1, &pointCloudVAO);
    glDeleteVertexArrays(1, &pointCloudVBO);
    return 0;
}