#ifndef SHADOWS_SHADOWS_INCLUDE_CAMERA_H_
#define SHADOWS_SHADOWS_INCLUDE_CAMERA_H_

#include "glm/mat4x4.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/transform.hpp"
#include "GL/glew.h"
#include "GLFW/glfw3.h"

class Camera {

 public:
    Camera(GLFWwindow * window, GLuint shaderProgramId, glm::vec3 eye, glm::vec3 at, glm::vec3 up);
    ~Camera();

  glm::mat4 getView();
  glm::mat4 getProjection();

  void updateShaderUniforms();
  void inputMoveUp();
  void inputMoveDown();
  void inputMoveLeft();
  void inputMoveRight();
  void updateMouse(const glm::vec2 &newMousePosition);

  void computeView();
  void computeProjection();

  void zoom(float factor);
  void strafe(float factor);
  void reset_aspect_ratio();
  void restoreDefaultEyeLocation();

 private:

  glm::mat4 _view;
  glm::mat4 _projection;
  glm::vec3 _eye;
  glm::vec3 _eyeDefault;
  glm::vec3 _at;
  glm::vec3 _up = glm::vec3(0,-1,0);
  glm::vec3 _viewDirection = glm::vec3(0, 0,1);
//  glm::vec3 _viewDirection = glm::vec3(-0.5453, -0.4389,0.9975);
  glm::vec3 _viewDirectionDefault = _viewDirection;

  GLuint _shaderProgramId = 0;
  GLuint _uniformViewPosition = 0;

  GLFWwindow * _w = nullptr;


};


#endif //SHADOWS_SHADOWS_INCLUDE_CAMERA_H_
