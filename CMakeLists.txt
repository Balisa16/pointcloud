cmake_minimum_required(VERSION 3.15)
project(pointcloud VERSION 1.0.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
# find_package(OpenGL REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

# Add the third-party directory
add_subdirectory(thirdparty/glfw)
add_subdirectory(thirdparty/glm)

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    # ${OPENGL_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/glm
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME}
    src/ebo.cc
    src/glad.c
    src/shader.cc
    src/stb.cc
    src/texture.cc
    src/vao.cc
    src/vbo.cc
    src/stb.cc
    src/camera.cc
)

target_link_libraries(${PROJECT_NAME} glfw ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

add_executable(main_test src/main.cpp)
add_executable(pc src/pc.cpp)

# Link GLFW to your executable
target_link_libraries(main_test PRIVATE
glfw
# -lGL
${PROJECT_NAME}
${PCL_LIBRARIES}
${OPENGL_LIBRARIES})

target_link_libraries(pc PRIVATE
glfw
# -lGL
${PROJECT_NAME}
${PCL_LIBRARIES}
${OPENGL_LIBRARIES})
