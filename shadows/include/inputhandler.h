#ifndef SHADOWS_SHADOWS_INCLUDE_INPUTHANDLER_H_
#define SHADOWS_SHADOWS_INCLUDE_INPUTHANDLER_H_

#include <iostream>
#include "GLFW/glfw3.h"
#include "app_state.h"
#include "camera.h"

class InputHandler {
 public:
  InputHandler(GLFWwindow * window, Camera *c) { _w = window;  _c = c; };
  ~InputHandler() { }

//  void registerCallbacks() {
//      glfwSetWindowSizeCallback(_w, window_size_callback);
//      glfwSetMouseButtonCallback(window,mouse_button_callback);
//      glfwSetScrollCallback(window,scroll_callback);
//      glfwSetCursorPosCallback(window,cursor_pos_callback);
//      glfwSetKeyCallback(window,key_callback);
//  }

  void handleKeys() {
    if (glfwGetKey(_w, GLFW_KEY_UP)) {
      _c->inputMoveUp();
    }

    if (glfwGetKey(_w, GLFW_KEY_DOWN)) {
      _c->inputMoveDown();
    }

    if (glfwGetKey(_w, GLFW_KEY_LEFT)) {
      _c->inputMoveLeft();
    }

    if (glfwGetKey(_w, GLFW_KEY_RIGHT)) {
      _c->inputMoveRight();
    }
    if (glfwGetKey(_w, GLFW_KEY_R)) {
        windowState.yaw = 0;
          windowState.pitch = 0;
          windowState.offset_x = 0;
          windowState.offset_y = 1;
          windowState.last_x = 0;
          windowState.last_y = 0;
    }
  }

    // When the window is resized, re-generate the perspective matrix (new aspect ratio)
    void window_size_callback(GLFWwindow* window, int newWidth, int newHeight) {
//    std::cout << "Window resized" << std::endl;
        float aspectRatio = (float) newWidth / (float) newHeight;
        _c->set_aspect_ratio(aspectRatio);
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
        if(fabs(windowState.yaw) > 360) {
            windowState.yaw = windowState.yaw = 0;
        }
        if(fabs(windowState.pitch) > 360) {
            windowState.pitch = windowState.pitch = 0;
        }
    }
  
 private:
  GLFWwindow * _w;
  Camera * _c;
};


#endif //SHADOWS_SHADOWS_INCLUDE_INPUTHANDLER_H_
