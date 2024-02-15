
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
#include <types.hpp>
// Function to read the contents of a file and return it as a string

int width = 800, height = 800;

int main()
{
    PCDReader parser("../sample/pointcloud1.pcd");
    Buffer buff;
    // parser += "../sample/pointcloud3.pcd";
    PCDFormat data = parser.get_data();
    buff = data;

    Window win("Point Cloud", width, height);
    GLFWwindow *window = win.get_window();

    Shader shader("../script/pc.vert", "../script/pc.frag");

    std::cout << "Point 0\n";
    CameraFrame cam_frame;
    cam_frame.add({1.0f, .0f, .0f}, {1.f, .0f, .0f, .0});
    cam_frame.add({2.0f, .0f, .0f}, {1.f, .0f, .0f, .0});
    cam_frame.add({3.0f, .0f, .0f}, {1.f, .0f, .0f, .0});

    GLuint vao, vbo;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);

    glBufferData(GL_ARRAY_BUFFER, (buff.size() * 6 + buff.start()) * sizeof(GLfloat), buff.data, GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (void *)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (void *)(3 * sizeof(GLfloat)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);

    std::cout << "Point 1\n";

    GLuint vao_line, vbo_line;
    glGenVertexArrays(2, &vao_line);
    glBindVertexArray(vao_line);

    glGenBuffers(2, &vbo_line);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_line);

    glBufferData(GL_ARRAY_BUFFER, cam_frame.max_size() * cam_frame.unit() * sizeof(GLfloat), cam_frame.data, GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, vbo_line);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (void *)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (void *)(3 * sizeof(GLfloat)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);

    std::cout << "Point 2\n";
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

    int counter = 4;
    while (!glfwWindowShouldClose(window))
    {
        counter--;
        win.clear();

        shader.Activate();

        Camera::Inputs(window);

        Camera::Matrix(45.0f, 0.1f, 100.0f, shader.ID, "camera_view_mat");

        if (counter == 3)
        {
            PCDReader parser2("../sample/pointcloud2.pcd");
            Buffer _temp_buff;
            _temp_buff = parser2.get_data();
            buff += _temp_buff;
            glBindBuffer(GL_ARRAY_BUFFER, vbo);
            glBufferData(GL_ARRAY_BUFFER, (buff.size() * 6 + buff.start()) * sizeof(GLfloat), buff.data, GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
        }
        else if (counter == 2)
        {
            PCDReader parser2("../sample/pointcloud3.pcd");
            Buffer _temp_buff;
            _temp_buff = parser2.get_data();
            buff += _temp_buff;
            glBindBuffer(GL_ARRAY_BUFFER, vbo);
            glBufferData(GL_ARRAY_BUFFER, (buff.size() * 6 + buff.start()) * sizeof(GLfloat), buff.data, GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
        }
        else if (counter == 1)
        {
            PCDReader parser2("../sample/pointcloud4.pcd");
            Buffer _temp_buff;
            _temp_buff = parser2.get_data();
            buff += _temp_buff;
            std::cout << buff.size() << std::endl;
            glBindBuffer(GL_ARRAY_BUFFER, vbo);
            glBufferData(GL_ARRAY_BUFFER, (buff.size() * 6 + buff.start()) * sizeof(GLfloat), buff.data, GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
        }

        std::cout << "Point 3\n";
        glDrawArrays(GL_LINES, 0, int(buff.start() / 6));
        glDrawArrays(GL_POINTS, buff.start() / 6 - 1, buff.size());

        glBindVertexArray(vao);

        // glDrawArrays(GL_POINTS, 0, cam_frame.size() * 18);
        // std::cout << "Point 4\n";

        // glBindVertexArray(vao_line);

        glfwSwapBuffers(window);
        // Take care of all GLFW events
        glfwPollEvents();
    }

    shader.Delete();
    // VAO1.Delete();
    // VBO1.Delete();
    glDeleteVertexArrays(1, &vao);
    glDeleteVertexArrays(1, &vbo);

    glDeleteVertexArrays(1, &vao_line);
    glDeleteVertexArrays(1, &vbo_line);

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}