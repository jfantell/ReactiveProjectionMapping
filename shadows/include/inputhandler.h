#ifndef SHADOWS_SHADOWS_INCLUDE_INPUTHANDLER_H_
#define SHADOWS_SHADOWS_INCLUDE_INPUTHANDLER_H_

#include <iostream>
#include "camera.h"
#include "GLFW/glfw3.h"

class InputHandler {
 public:
  static void set_camera(Camera *c);
  static void key_callback(GLFWwindow* window, int key, int sancode, int action, int mods);
  static void window_size_callback(GLFWwindow* window, int newWidth, int newHeight);
  static void mouse_button_callback(GLFWwindow* window,int button, int action, int mods);
  static void scroll_callback(GLFWwindow* window,double xoffset, double yoffset);
  static void cursor_pos_callback(GLFWwindow* window, double x, double y);
 private:
  static Camera* _c;
};


#endif //SHADOWS_SHADOWS_INCLUDE_INPUTHANDLER_H_
