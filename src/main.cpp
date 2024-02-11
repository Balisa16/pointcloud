#include <iostream>
// #include <GL/glew.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <glad.h>
#include <GLFW/glfw3.h>

// Function to draw the point cloud
void drawPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    glPointSize(2.0f);
    glBegin(GL_POINTS);

    for (const auto &point : cloud->points)
    {
        if (!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z))
        {
            glColor3ub(255, 0, 0);
            glVertex3f(point.x, point.y, point.z);
        }
    }

    glEnd();
}

GLFWwindow *initWindow(int width, int height, const char *title)
{
    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        exit(EXIT_FAILURE);
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Disable resize
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

    GLFWwindow *window = glfwCreateWindow(width, height, title, NULL, NULL);
    if (!window)
    {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glfwMakeContextCurrent(window);

    glEnable(GL_DEPTH_TEST);
    glViewport(0, 0, width, height);

    return window;
}

int main()
{
    // Read PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../sample/ICRA_willow_challenge/T_02/cloud_0000000000.pcd", *cloud) == -1)
    {
        std::cerr << "Couldn't read PCD file" << std::endl;
        return -1;
    }
    for (auto &data : cloud->points)
    {
    }

    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    GLFWwindow *window = initWindow(640, 480, "Point Cloud Viewer");
    if (!window)
    {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }

    while (!glfwWindowShouldClose(window))
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Draw point cloud
        drawPointCloud(cloud);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Cleanup
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
