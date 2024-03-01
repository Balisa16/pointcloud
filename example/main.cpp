
#include <pointcloud.hpp>

int width = 800, height = 800;

int main()
{
    Pointcloud buff;
    CameraFrame cam_frame;

    Window win("Pointcloud", width, height);
    GLFWwindow *window = win.get_window();

    Shader shader("script/pc.vert", "script/pc.frag");

    // Pointcloud VAO and VBO
    VAO pc_vao;
    pc_vao.bind();

    VBO pc_vbo(buff.data, buff.size() * 6 * sizeof(GLfloat));
    pc_vbo.set_vertices(0, 1, 3, 3);
    pc_vbo.unbind();
    pc_vao.unbind();

    // Camera Frame VAO and VBO
    VAO frame_vao;
    frame_vao.bind();

    VBO frame_vbo(cam_frame.data, cam_frame.array_len() * sizeof(GLfloat));
    frame_vbo.set_vertices(0, 1, 3, 3);
    frame_vbo.unbind();
    frame_vao.unbind();

    glEnable(GL_DEPTH_TEST);

    Camera::init(window, width, height, glm::vec3(0.0f, 0.0f, 0.0f));

    FileHandler::read("sample");

    while (!glfwWindowShouldClose(window))
    {
        win.clear();

        shader.activate();

        Camera::Inputs(window);

        Camera::Matrix(45.0f, 0.1f, 100.0f, shader.get_id(), "camera_view_mat");

        FileHandler::sync();

        // Bind Pointcloud

        if (FileHandler::is_new_data())
        {
            Pointcloud _new_data;
            FileHandler::get_data(_new_data);
            FileHandler::get_camera_frame(cam_frame);

            buff += _new_data;

            // Bind Pointcloud
            glBindBuffer(GL_ARRAY_BUFFER, pc_vbo.get_id());
            glBufferData(GL_ARRAY_BUFFER, buff.size() * 6 * sizeof(GLfloat), buff.data, GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, 0);

            // Bind Camera Frame
            glBindBuffer(GL_ARRAY_BUFFER, frame_vbo.get_id());
            glBufferData(GL_ARRAY_BUFFER, cam_frame.array_len() * sizeof(GLfloat), cam_frame.data, GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
        }

        glBindVertexArray(pc_vao.get_id());
        glDrawArrays(GL_POINTS, 0, buff.size());
        glBindVertexArray(0);

        glBindVertexArray(frame_vbo.get_id());
        glDrawArrays(GL_LINES, 0, cam_frame.array_len() / 6);
        glBindVertexArray(0);

        glfwSwapBuffers(window);

        glfwPollEvents();
    }
    shader.Delete();
    return 0;
}