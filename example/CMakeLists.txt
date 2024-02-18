add_executable(main_test main.cpp)
add_executable(pc pc.cpp)
# add_executable(test_prog test/main.cpp)

# # Link GLFW to your executable
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