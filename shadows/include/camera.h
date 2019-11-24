#ifndef SHADOWS_SHADOWS_INCLUDE_CAMERA_H_
#define SHADOWS_SHADOWS_INCLUDE_CAMERA_H_

#include "glm/mat4x4.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "GL/glew.h"
#include "GLFW/glfw3.h"

class Camera {

 public:
  Camera(GLFWwindow * window, GLuint shaderProgramId, float fov_degrees, float aspectratio, glm::vec3 worldlocation, glm::vec3 lookat);
  ~Camera();

  glm::mat4 getView();
  glm::mat4 getProjection();

  void lookAt(glm::vec3 lookAt);
  void setWorldLocation(glm::vec3 worldLocation);
  glm::vec3 getWorldLocation();

  void setWorldX(float x) { _worldLocation.x = x; computeView(); }
  void setWorldY(float y) { _worldLocation.y = y; computeView(); }
  void setWorldZ(float z) { _worldLocation.z = z; computeView(); }

  float getWorldX() { return _worldLocation.x; }
  float getWorldY() { return _worldLocation.y; }
  float getWorldZ() { return _worldLocation.z; }

  void updateShaderUniforms();
  void inputMoveUp();
  void inputMoveDown();
  void inputMoveLeft();
  void inputMoveRight();

  void computeView();
  void computeProjection();
  void set_aspect_ratio(float aspect);
  void moveWorldLocation();
  void restoreDefaultWorldLocation();

 private:

  glm::mat4 _view;
  glm::mat4 _projection;
  glm::vec3 _worldLocation;
  glm::vec3 _worldLocationDefault;
  glm::vec3 _lookAt;
  glm::vec3 _up = glm::vec3(0,-1,0);

  float _fovRadians;
  float _aspectratio;

  GLuint _shaderProgramId = 0;
  GLuint _uniformViewPosition = 0;

  GLFWwindow * _w = nullptr;


};


#endif //SHADOWS_SHADOWS_INCLUDE_CAMERA_H_
