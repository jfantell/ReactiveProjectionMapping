//
// Callbacks are triggered by user input, i.e. user resizing the OpenGL window
// Callbacks allow the user to interactively modify the OpenGL canvas
// Currently, users can "zoom" in and out and "rotate" the view window
// To reset the view window (to defaults) press "R"
//

#ifndef REALSENSE2_CALLBACKS_H
#define REALSENSE2_CALLBACKS_H

#include <GLFW/glfw3.h>

void window_size_callback(GLFWwindow* window, int newWidth, int newHeight);
void mouse_button_callback(GLFWwindow* window,int button, int action, int mods);
void scroll_callback(GLFWwindow* window,double xoffset, double yoffset);
void cursor_pos_callback(GLFWwindow* window, double x, double y);
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);

#endif //REALSENSE2_CALLBACKS_H
