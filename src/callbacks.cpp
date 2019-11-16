//
// Callbacks are triggered by user input, i.e. user resizing the OpenGL window
// Callbacks allow the user to interactively modify the OpenGL canvas
// Currently, users can "zoom" in and out and "rotate" the view window
// To reset the view window (to defaults) press "R"
//

#include "app_state.h"
#include <GLFW/glfw3.h>
#include <iostream>
#include "transformations.h"

// When the window is resized, re-generate the perspective matrix (new aspect ratio)
void window_size_callback(GLFWwindow* window, int newWidth, int newHeight) {
//    std::cout << "Window resized" << std::endl;
    perspective_matrix(window);
}
void mouse_button_callback(GLFWwindow* window,int button, int action, int mods){
    if(button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
//        std::cout << "Button pressed" << std::endl;
        windowState.ml = true;
    }
    if(button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE){
//        std::cout << "Button released" << std::endl;
        windowState.ml = false;
    }
}
void scroll_callback(GLFWwindow* window,double xoffset, double yoffset){
    windowState.offset_x -= static_cast<float>(xoffset);
    windowState.offset_y -= static_cast<float>(yoffset);
//    std::cout << "Zooming" << std::endl;
}
void cursor_pos_callback(GLFWwindow* window, double x, double y){
    if (windowState.ml)
        {
            windowState.yaw -= (x - windowState.last_x);
            windowState.pitch += (y - windowState.last_y);
        }
        windowState.last_x = x;
        windowState.last_y = y;
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods){
    if (key == GLFW_KEY_R && action == GLFW_PRESS){
        windowState.yaw = 0;
        windowState.pitch = 0;
        windowState.offset_x = 0;
        windowState.offset_y = 1;
        windowState.last_x = 0;
        windowState.last_y = 0;
    }
}