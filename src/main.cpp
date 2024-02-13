
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

struct Buffer
{
private:
    const uint64_t data_limit = 600000;
    uint64_t _size;

public:
    GLfloat *data;
    Buffer() : data(new GLfloat[data_limit * 6]), _size(0) {}
    void operator=(PCDFormat &new_data)
    {
        clear();
        int __cnt = 0;
        for (uint64_t i = 0; i < new_data.num_points; ++i)
        {
            if (__cnt >= data_limit)
            {
                _size = data_limit;
                __cnt = 0;
            }
            data[__cnt * 6] = new_data.points[i].x;
            data[__cnt * 6 + 1] = new_data.points[i].y;
            data[__cnt * 6 + 2] = new_data.points[i].z;
            data[__cnt * 6 + 3] = new_data.points[i].r;
            data[__cnt * 6 + 4] = new_data.points[i].g;
            data[__cnt * 6 + 5] = new_data.points[i].b;
            __cnt++;
        }
        if (_size < data_limit)
            _size = __cnt;
    }

    void operator=(Buffer &new_data)
    {
        clear();
        int __cnt = 0;
        for (uint64_t i = 0; i < new_data.size(); ++i)
        {
            if (__cnt >= data_limit)
            {
                _size = data_limit;
                __cnt = 0;
            }

            data[__cnt * 6] = new_data.data[i * 6];
            data[__cnt * 6 + 1] = new_data.data[i * 6 + 1];
            data[__cnt * 6 + 2] = new_data.data[i * 6 + 2];
            data[__cnt * 6 + 3] = new_data.data[i * 6 + 3];
            data[__cnt * 6 + 4] = new_data.data[i * 6 + 4];
            data[__cnt * 6 + 5] = new_data.data[i * 6 + 5];
            __cnt++;
        }

        if (_size < data_limit)
            _size = __cnt;
    }

    void operator+=(PCDFormat &new_data)
    {
        int __cnt = _size;
        for (uint64_t i = 0; i < new_data.num_points; ++i)
        {
            if (__cnt >= data_limit)
            {
                _size = data_limit;
                __cnt = 0;
            }
            data[(__cnt + i) * 6] = new_data.points[i].x;
            data[(__cnt + i) * 6 + 1] = new_data.points[i].y;
            data[(__cnt + i) * 6 + 2] = new_data.points[i].z;
            data[(__cnt + i) * 6 + 3] = new_data.points[i].r;
            data[(__cnt + i) * 6 + 4] = new_data.points[i].g;
            data[(__cnt + i) * 6 + 5] = new_data.points[i].b;
            __cnt++;
        }
        if (_size < data_limit)
            _size = __cnt;
    }

    Buffer operator+(const Buffer &buffer)
    {
        Buffer result;
        result = *this;

        // Copy data from the second buffer
        uint64_t __cnt = result._size;
        for (uint64_t i = 0; i < buffer._size; ++i)
        {
            // Check if data too large
            if (__cnt >= data_limit)
            {
                result._size = data_limit;
                __cnt = 0;
            }

            result.data[(__cnt + i) * 6] = buffer.data[i * 6];
            result.data[(__cnt + i) * 6 + 1] = buffer.data[i * 6 + 1];
            result.data[(__cnt + i) * 6 + 2] = buffer.data[i * 6 + 2];
            result.data[(__cnt + i) * 6 + 3] = buffer.data[i * 6 + 3];
            result.data[(__cnt + i) * 6 + 4] = buffer.data[i * 6 + 4];
            result.data[(__cnt + i) * 6 + 5] = buffer.data[i * 6 + 5];
            __cnt++;
        }

        if (result._size < data_limit)
            result._size = __cnt;

        return result;
    }

    void clear()
    {
        if (data != nullptr)
            delete[] data;

        data = new GLfloat[data_limit * 6];
        _size = 0;
    }

    uint64_t size() const
    {
        return _size;
    }

    GLfloat *get_data()
    {
        return data;
    }
};

int main()
{
    PCDReader parser("../sample/pointcloud1.pcd");
    Buffer buff;
    // parser += "../sample/pointcloud3.pcd";
    PCDFormat data = parser.get_data();
    buff = data;

    for (uint64_t i = 0; i < buff.size(); ++i)
        std::cout << i << '\t' << buff.data[i * 6] << " " << buff.data[i * 6 + 1] << " " << buff.data[i * 6 + 2] << std::endl;

    Window win("Point Cloud", width, height);
    GLFWwindow *window = win.get_window();

    Shader shader("../script/pc.vert", "../script/pc.frag");

    GLuint vao, vbo;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);

    glBufferData(GL_ARRAY_BUFFER, data.num_points * 6 * sizeof(GLfloat), data.gl_data, GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);

    // VAO VAO1;
    // VAO1.Bind();

    // VBO VBO1(data.gl_data, data.num_points * 6 * sizeof(GLfloat));

    // VAO1.LinkAttrib(VBO1, 0, 3, GL_FLOAT, 6 * sizeof(float), (void *)0);
    // VAO1.LinkAttrib(VBO1, 1, 3, GL_FLOAT, 6 * sizeof(float), (void *)(3 * sizeof(float)));

    // VAO1.Unbind();
    // VBO1.Unbind();

    glEnable(GL_DEPTH_TEST);

    Camera::init(window, width, height, glm::vec3(0.0f, 0.0f, 2.0f));

    // Main loop

    int counter = 500;
    while (!glfwWindowShouldClose(window))
    {
        counter--;
        win.clear();

        shader.Activate();

        Camera::Inputs(window);

        Camera::Matrix(45.0f, 0.1f, 100.0f, shader.ID, "camera_view_mat");

        if (counter == 0)
        {
            parser += "../sample/pointcloud3.pcd";
            data = parser.get_data();
            glBindBuffer(GL_ARRAY_BUFFER, vbo);
            glBufferData(GL_ARRAY_BUFFER, data.num_points * 6 * sizeof(GLfloat), data.gl_data, GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
        }

        glDrawArrays(GL_POINTS, 0, data.num_points);

        // VAO1.Bind();
        glBindVertexArray(vao);

        glfwSwapBuffers(window);
        // Take care of all GLFW events
        glfwPollEvents();
    }

    shader.Delete();
    // VAO1.Delete();
    // VBO1.Delete();
    glDeleteVertexArrays(1, &vao);
    glDeleteVertexArrays(1, &vbo);
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}