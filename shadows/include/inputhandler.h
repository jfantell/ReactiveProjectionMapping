#ifndef SHADOWS_SHADOWS_INCLUDE_INPUTHANDLER_H_
#define SHADOWS_SHADOWS_INCLUDE_INPUTHANDLER_H_

#include <iostream>
#include "GLFW/glfw3.h"

enum {
  UP = 0,
  DOWN = 1,
  LEFT = 2,
  RIGHT = 3,
  NONE = 4,
};

struct debounce {
  bool UP_ENABLE = true;
  bool DOWN_ENABLE = true;
  bool LEFT_ENABLE = true;
  bool RIGHT_ENABLE = true;
};

class InputHandler {
 public:
  InputHandler(GLFWwindow * window) { _w = window; };
  ~InputHandler() { }

  unsigned int handle() {
    if (glfwGetKey(_w, GLFW_KEY_UP)) {
      _debounce.UP_ENABLE = false;
      return UP;
    }
    else if (!glfwGetKey(_w, GLFW_KEY_UP) && !_debounce.UP_ENABLE) {
      _debounce.UP_ENABLE = true;
    }
    if (glfwGetKey(_w, GLFW_KEY_DOWN)) {
      _debounce.DOWN_ENABLE = false;
      return DOWN;
    }
    else if (!glfwGetKey(_w, GLFW_KEY_DOWN) && !_debounce.DOWN_ENABLE) {
      _debounce.DOWN_ENABLE = true;
    }
    if (glfwGetKey(_w, GLFW_KEY_LEFT)) {
      _debounce.LEFT_ENABLE = false;
      return LEFT;
    }
    else if (!glfwGetKey(_w, GLFW_KEY_LEFT) && !_debounce.LEFT_ENABLE) {
      _debounce.LEFT_ENABLE = true;
    }
    if (glfwGetKey(_w, GLFW_KEY_RIGHT)) {
      _debounce.RIGHT_ENABLE = false;
      return RIGHT;
    }
    else if (!glfwGetKey(_w, GLFW_KEY_RIGHT) && !_debounce.RIGHT_ENABLE) {
      _debounce.RIGHT_ENABLE = true;
    }
    return NONE;
  }
  
 private:
  GLFWwindow * _w;
  debounce _debounce;

};


#endif //SHADOWS_SHADOWS_INCLUDE_INPUTHANDLER_H_
