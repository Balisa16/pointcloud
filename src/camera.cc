#include <camera.hpp>

void Camera::camera_init(GLFWwindow *window, int width, int height, glm::vec3 position)
{
    this->width = width;
    this->height = height;
    this->window = window;
    this->Position = position;
    glfwSetScrollCallback(window, scroll_callback);
}

void Camera::scroll_callback(GLFWwindow *window, double xoffset, double yoffset)
{
    get().mouse_scroll(yoffset > 0);
}

void Camera::camera_matrix(float FOVdeg, float nearPlane, float farPlane, GLuint shader_id, const char *uniform)
{
    glm::mat4 view = glm::mat4(1.0f);
    glm::mat4 projection = glm::mat4(1.0f);

    view = glm::lookAt(Position, Position + Orientation, Up);
    projection = glm::perspective(glm::radians(FOVdeg), (float)width / height, nearPlane, farPlane);

    // Exports the camera matrix to the Vertex Shader
    glUniformMatrix4fv(glGetUniformLocation(shader_id, uniform), 1, GL_FALSE, glm::value_ptr(projection * view));
}

void Camera::mouse_scroll(bool up)
{
    if (up)
        Position += speed * Orientation;
    else
        Position -= speed * Orientation;
}

void Camera::camera_inputs(GLFWwindow *window)
{
    // Handles key inputs
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
    {
        Orientation = glm::rotate(Orientation, glm::radians(-.6f), glm::vec3(0.0f, 0.0f, -1.0f));
        // Position += speed * Orientation;
    }
    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
    {
        Position += speed * -glm::normalize(glm::cross(Orientation, Up));
    }
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
    {
        Orientation = glm::rotate(Orientation, glm::radians(.6f), glm::vec3(0.0f, 0.0f, -1.0f));
        // Position += speed * -Orientation;
    }
    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
    {
        Position += speed * glm::normalize(glm::cross(Orientation, Up));
    }
    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
    {
        Position += speed * Up;
    }
    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
    {
        Position += speed * -Up;
    }
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
    {
        speed = 0.4f;
    }
    else if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_RELEASE)
    {
        speed = 0.1f;
    }

    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
    {
        Orientation = glm::rotate(Orientation, glm::radians(.4f), glm::vec3(0.0f, 1.0f, 0.0f));
    }

    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
    {
        Orientation = glm::rotate(Orientation, glm::radians(-.4f), glm::vec3(0.0f, 1.0f, 0.0f));
    }

    // Handles mouse inputs
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
    {
        // Hides mouse cursor
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);

        // Prevents camera from jumping on the first click
        if (firstClick)
        {
            glfwSetCursorPos(window, (width / 2), (height / 2));
            firstClick = false;
        }

        // Stores the coordinates of the cursor
        double mouseX;
        double mouseY;
        // Fetches the coordinates of the cursor
        glfwGetCursorPos(window, &mouseX, &mouseY);

        // Normalizes and shifts the coordinates of the cursor such that they begin in the middle of the screen
        // and then "transforms" them into degrees
        float rotX = sensitivity * (float)(mouseY - (height / 2)) / height;
        float rotY = sensitivity * (float)(mouseX - (width / 2)) / width;

        // Calculates upcoming vertical change in the Orientation
        glm::vec3 newOrientation = glm::rotate(Orientation, glm::radians(rotX), glm::normalize(glm::cross(Orientation, Up)));

        // Decides whether or not the next vertical Orientation is legal or not
        if (abs(glm::angle(newOrientation, Up) - glm::radians(90.0f)) <= glm::radians(85.0f))
        {
            Orientation = newOrientation;
        }

        // Rotates the Orientation left and right
        Orientation = glm::rotate(Orientation, glm::radians(rotY), Up);

        // Sets mouse cursor to the middle of the screen so that it doesn't end up roaming around
        glfwSetCursorPos(window, (width / 2), (height / 2));
    }
    else if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_RELEASE)
    {
        // Unhides cursor since camera is not looking around anymore
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        // Makes sure the next time the camera looks around it doesn't jump
        firstClick = true;
    }
}