add_executable(test1 main.cpp)

target_include_directories(test1 PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/thirdparty/glm)

target_link_libraries(test1 PRIVATE
    glfw
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${PROJECT_NAME})