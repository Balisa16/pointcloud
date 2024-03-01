#pragma once

#include <glad.h>
#include <GLFW/glfw3.h>
#include <GL/glut.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

// STL Library
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>

#include <window.hpp>
#include <texture.hpp>
#include <vao.hpp>
#include <vbo.hpp>
#include <ebo.hpp>
#include <camera.hpp>
#include <shader.hpp>
#include <reader.hpp>
#include <filehandler.hpp>