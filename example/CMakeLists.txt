add_executable(main_test main.cpp)
# add_executable(pc pc.cpp)

target_include_directories(main_test PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/thirdparty/glm)

target_link_libraries(main_test PRIVATE
    glfw
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${PROJECT_NAME})

# target_include_directories(pc PRIVATE
#     ${PCL_INCLUDE_DIRS}
#     ${CMAKE_SOURCE_DIR}/include
#     ${CMAKE_SOURCE_DIR}/thirdparty/glm)

# target_link_libraries(pc PRIVATE ${PROJECT_NAME})
