cmake_minimum_required(VERSION 3.15)
project(pointcloud VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

add_subdirectory(thirdparty/glfw)
add_subdirectory(thirdparty/glm)

add_library(${PROJECT_NAME} SHARED
    src/transform.cc
    src/ebo.cc
    src/glad.c
    src/shader.cc
    src/stb.cc
    src/texture.cc
    src/vao.cc
    src/vbo.cc
    src/stb.cc
    src/camera.cc
    src/window.cc
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/thirdparty/glm)

target_link_libraries(${PROJECT_NAME} PRIVATE
    glfw
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES})

add_subdirectory(example)