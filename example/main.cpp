
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

#include <future>
#include <boost/filesystem.hpp>
// Function to read the contents of a file and return it as a string

int width = 800, height = 800;

class FileHandler
{
public:
    static FileHandler &get_instance()
    {
        static FileHandler instance;
        return instance;
    }

    static bool read(std::string foldername)
    {
        return get_instance().Iread(foldername);
    }

    static void sync()
    {
        get_instance().Isync();
    }

private:
    bool is_run_task = false;
    std::future<int> task_result;
    std::string folder_name = "";
    std::vector<std::string> file_list;

    bool Iread(std::string foldername)
    {
        if (!boost::filesystem::exists(foldername))
        {
            std::cout << "File doesn't exist.\n";
            return false;
        }

        for (boost::filesystem::directory_iterator it(foldername); it != boost::filesystem::directory_iterator(); ++it)
            if (boost::filesystem::is_regular_file(it->status()) && it->path().extension() == ".pcd")
                file_list.push_back(it->path().string());

        std::cout << "Found " << file_list.size() << " pcd files" << std::endl;

        folder_name = foldername;
    }

    void Isync()
    {
        if (folder_name == "" || std::empty(folder_name))
        {
            std::cout << "Please set the folder name first" << std::endl;
            return;
        }

        if (!is_run_task && file_list.size())
        {
            std::string file_name = file_list.front();

            // Remove the first element
            file_list.erase(file_list.begin());
            task_result = load_file_async(file_name);
            is_run_task = true;
        }

        if (task_result.valid() && task_result.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
        {
            int res = task_result.get();
            std::cout << "Async task is Ready : " << res << std::endl;
            is_run_task = false;
        }
    }

    std::future<int> load_file_async(std::string filename)
    {
        return std::async(std::launch::async, [filename]()
                          {
                            std::cout << "Load : " << filename << std::flush;
                            // for(int i = 0; i < 50; i++)
                            // {
                            //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
                            // }
                            PCDReader parser(filename);
                            PCDFormat data = parser.get_data();

                            std::cout << " [OK]" << std::endl;
                            return 0; });
    }

    FileHandler(const FileHandler &) = delete;
    FileHandler(FileHandler &&) = delete;
    FileHandler &operator=(const FileHandler &) = delete;
    FileHandler &operator=(FileHandler &&) = delete;

    FileHandler() = default;
    ~FileHandler() = default;
};

// std::future<int> load_file_async(std::string filename)
// {
//     return std::async(std::launch::async, [filename]()
//                       {
//                         std::cout << "Load : " << filename << std::flush;
//                         // for(int i = 0; i < 50; i++)
//                         // {
//                         //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
//                         // }
//                         PCDReader parser(filename);
//                         PCDFormat data = parser.get_data();

//                         std::cout << " [OK]" << std::endl;
//                         return 0; });
// }

int main()
{
    PCDReader parser("../../sample/pointcloud1.pcd");
    Buffer buff;
    parser += "../../sample/pointcloud3.pcd";
    PCDFormat data = parser.get_data();
    buff = data;

    // PCDFormat data = parser.get_data();

    Window win("Point Cloud", width, height);
    GLFWwindow *window = win.get_window();

    Shader shader("../../script/pc.vert", "../../script/pc.frag");

    GLuint vao, vbo;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);

    glBufferData(GL_ARRAY_BUFFER, data.num_points * 6 * sizeof(GLfloat), data.gl_data, GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (void *)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (void *)(3 * sizeof(GLfloat)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

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

        // if (counter == 3)
        // {
        //     result_task = load_file_async("../../sample/pointcloud2.pcd");
        //     // Buffer _temp_buff;
        //     // _temp_buff = parser2.get_data();
        //     // buff += _temp_buff;
        //     // glBindBuffer(GL_ARRAY_BUFFER, vbo);
        //     // glBufferData(GL_ARRAY_BUFFER, (buff.size() * 6 + buff.start()) * sizeof(GLfloat), buff.data, GL_STATIC_DRAW);
        //     // glBindBuffer(GL_ARRAY_BUFFER, 0);
        // }
        // else if (counter == 2)
        // {
        //     PCDReader parser3("../../sample/pointcloud3.pcd");
        //     // Buffer _temp_buff;
        //     // _temp_buff = parser3.get_data();
        //     // buff += _temp_buff;
        //     // glBindBuffer(GL_ARRAY_BUFFER, vbo);
        //     // glBufferData(GL_ARRAY_BUFFER, (buff.size() * 6 + buff.start()) * sizeof(GLfloat), buff.data, GL_STATIC_DRAW);
        //     // glBindBuffer(GL_ARRAY_BUFFER, 0);
        // }
        // else if (counter == 1)
        // {
        //     PCDReader parser4("../../sample/pointcloud4.pcd");
        //     // Buffer _temp_buff;
        //     // _temp_buff = parser4.get_data();
        //     // buff += _temp_buff;
        //     // glBindBuffer(GL_ARRAY_BUFFER, vbo);
        //     // glBufferData(GL_ARRAY_BUFFER, (buff.size() * 6 + buff.start()) * sizeof(GLfloat), buff.data, GL_STATIC_DRAW);
        //     // glBindBuffer(GL_ARRAY_BUFFER, 0);
        // }

        // if (!is_run_task && counter)
        // {
        //     std::cout << "Run Task : " << counter << std::endl;
        //     result_task = load_file_async("../../sample/pointcloud" + std::to_string(counter) + ".pcd");
        //     is_run_task = true;
        //     counter--;
        // }

        // if (result_task.valid() && result_task.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
        // {
        //     int res = result_task.get();
        //     std::cout << "Async task is Ready : " << res << std::endl;
        //     is_run_task = false;
        // }

        // if(result_task.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
        // {
        //     std::cout << "Async task is Ready" << std::endl;
        // }

        glBindVertexArray(vao);

        FileHandler::sync();

        // glDrawArrays(GL_LINES, 0, int(buff.start() / 6));

        // glDrawArrays(GL_POINTS, 0, data.num_points);

        glDrawArrays(GL_LINES, 0, int(buff.start() / 6));

        glDrawArrays(GL_POINTS, buff.start() / 6 - 1, buff.size());

        glfwSwapBuffers(window);
        // Take care of all GLFW events
        glfwPollEvents();
    }
    shader.Delete();
    // VAO1.Delete();
    // VBO1.Delete();

    glDeleteVertexArrays(1, &vao);
    glDeleteVertexArrays(1, &vbo);
    return 0;
}