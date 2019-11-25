//
// Created by John Fantell on 11/24/19.
//

#include "inputhandler.h"
#include "app_state.h"

//Define static variable in cpp file
Camera* InputHandler:: _c;

void InputHandler::set_camera(Camera *c) {
    _c = c;
}

void InputHandler::key_callback(GLFWwindow *window, int key, int sancode, int action, int mods) {
    if (key == GLFW_KEY_UP && action == GLFW_PRESS) {
        _c->inputMoveUp();
    }
    if (key == GLFW_KEY_DOWN && action == GLFW_PRESS) {
        _c->inputMoveDown();
    }
    if (key == GLFW_KEY_LEFT && action == GLFW_PRESS) {
        _c->inputMoveLeft();
    }
    if (key == GLFW_KEY_RIGHT && action == GLFW_PRESS) {
        _c->inputMoveRight();
    }
    if (key == GLFW_KEY_R && action == GLFW_PRESS) {
        std::cout << " Reset " << std::endl;
        windowState.yaw = 0;
        windowState.pitch = 0;
        windowState.offset_x = 0;
        windowState.offset_y = 1;
        windowState.last_x = 0;
        windowState.last_y = 0;
        _c->restoreDefaultWorldLocation();
    }
}

// When the window is resized, re-generate the perspective matrix (new aspect ratio)
void InputHandler::window_size_callback(GLFWwindow *window, int newWidth, int newHeight) {
    windowState.width = newWidth;
    windowState.height = newHeight;
    _c->reset_aspect_ratio();
}

void InputHandler::mouse_button_callback(GLFWwindow *window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        windowState.ml = true;
    }
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
        windowState.ml = false;
    }
}

void InputHandler::scroll_callback(GLFWwindow *window, double xoffset, double yoffset) {
    windowState.offset_x -= static_cast<float>(xoffset);
    windowState.offset_y -= static_cast<float>(yoffset);
    std::cout << "Zoom" << windowState.offset_y << std::endl;
    _c->moveWorldLocation();
}

void InputHandler::cursor_pos_callback(GLFWwindow *window, double x, double y) {
    if (windowState.ml) {
        windowState.yaw -= (x - windowState.last_x);
        windowState.pitch += (y - windowState.last_y);
        std::cout << "Rotate - Yaw: " << windowState.yaw <<  " Pitch:  " << windowState.pitch <<std::endl;
        _c->moveWorldLocation();
    }
    windowState.last_x = x;
    windowState.last_y = y;
    if (fabs(windowState.yaw) > 360) {
        windowState.yaw = windowState.yaw = 0;
    }
    if (fabs(windowState.pitch) > 360) {
        windowState.pitch = windowState.pitch = 0;
    }
}