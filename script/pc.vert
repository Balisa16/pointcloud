#version 330 core

layout (location = 0) in vec3 a_position;
layout (location = 1) in vec3 a_color;

out vec3 color;

uniform mat4 camera_view_mat;
// uniform vec3 point_take_position;
// uniform vec4 point_take_orientation;

void main() {
    // Construct the rotation matrix based on the quaternion orientation
    // mat3 rotation_mat = mat3(1.0 - 2.0 * point_take_orientation.y * point_take_orientation.y - 2.0 * point_take_orientation.z * point_take_orientation.z,
    //                         2.0 * point_take_orientation.x * point_take_orientation.y + 2.0 * point_take_orientation.w * point_take_orientation.z,
    //                         2.0 * point_take_orientation.x * point_take_orientation.z - 2.0 * point_take_orientation.w * point_take_orientation.y,

    //                         2.0 * point_take_orientation.x * point_take_orientation.y - 2.0 * point_take_orientation.w * point_take_orientation.z,
    //                         1.0 - 2.0 * point_take_orientation.x * point_take_orientation.x - 2.0 * point_take_orientation.z * point_take_orientation.z,
    //                         2.0 * point_take_orientation.y * point_take_orientation.z + 2.0 * point_take_orientation.w * point_take_orientation.x,

    //                         2.0 * point_take_orientation.x * point_take_orientation.z + 2.0 * point_take_orientation.w * point_take_orientation.y,
    //                         2.0 * point_take_orientation.y * point_take_orientation.z - 2.0 * point_take_orientation.w * point_take_orientation.x,
    //                         1.0 - 2.0 * point_take_orientation.x * point_take_orientation.x - 2.0 * point_take_orientation.y * point_take_orientation.y);

    // Translate and rotate the point
    // vec3 real_position = rotation_mat * (a_position - point_take_position) + point_take_position;

    // Set the color output
    color = a_color;

    // Set the gl_Position using camera view matrix
    // gl_Position = camera_view_mat * vec4(real_position, 1.0);
    gl_Position = camera_view_mat * vec4(a_position, 1.0);
    gl_PointSize = 5.0;
}
