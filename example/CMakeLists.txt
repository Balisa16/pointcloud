add_executable(main_test main.cpp)
add_executable(pc pc.cpp)
target_link_libraries(main_test PRIVATE
    glfw
    ${PROJECT_NAME}
    ${PCL_LIBRARIES}
    ${OPENGL_LIBRARIES})

target_link_libraries(pc PRIVATE
    glfw
    ${PROJECT_NAME}
    ${PCL_LIBRARIES}
    ${OPENGL_LIBRARIES})