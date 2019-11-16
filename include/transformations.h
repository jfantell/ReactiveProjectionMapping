// Create transformation matrices required to render the RealSense data
// The transformation matrices will be computed in the C++ program and
// sent to the vertex shader


#ifndef REALSENSE2_TRANSFORMATIONS_H
#define REALSENSE2_TRANSFORMATIONS_H

#include "app_state.h"
#include <GLFW/glfw3.h>
#include <glm/ext.hpp>

void perspective_matrix(GLFWwindow* win);
void model_view_matrix();

#endif //REALSENSE2_TRANSFORMATIONS_H
