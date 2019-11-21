#include "camera.h"
#include <iostream>
#include "GLFW/glfw3.h"

#define MOVEMENT_DELTA 0.02f


Camera::Camera(GLFWwindow * window, GLuint shaderProgramId, float fov_degrees, float aspectratio, glm::vec3 worldLocation, glm::vec3 lookAt) {
  _shaderProgramId = shaderProgramId;
  _fovRadians = glm::radians(fov_degrees);
  _aspectratio = aspectratio;
  _worldLocation = worldLocation;
  _lookAt = lookAt;

  _uniformViewPosition = glGetUniformLocation(shaderProgramId, "viewPosition");

  _w = window;

  updateShaderUniforms();

  computeView();
  computeProjection();

}

Camera::~Camera() { } // Empty destructor

glm::mat4 Camera::getView() {
  return _view;
}

glm::mat4 Camera::getProjection() {
  return _projection;
}

void Camera::lookAt(glm::vec3 lookAt) {
  _lookAt = lookAt;
  computeView();
}

void Camera::setWorldLocation(glm::vec3 worldLocation) {
  _worldLocation = worldLocation;
  computeView();
  computeProjection();
}

glm::vec3 Camera::getWorldLocation() {
  return _worldLocation;
}

void Camera::computeView() {
  _view = glm::lookAt(
      _worldLocation, // Camera is at (4,3,3), in World Space
      _lookAt, // and looks at the origin
      _up  // Head is up (set to 0,-1,0 to look upside-down)
  );
}

void Camera::computeProjection() {
  // Projection matrix : 45° Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
  _projection = glm::perspective(_fovRadians, _aspectratio, 0.1f, 100.0f);

}

void Camera::updateShaderUniforms() {
  glUniform3f(_uniformViewPosition, _worldLocation.x, _worldLocation.y, _worldLocation.z);
}


void Camera::inputMoveUp() {
  _worldLocation -= glm::normalize(_worldLocation) * MOVEMENT_DELTA;
  computeView();
}

void Camera::inputMoveDown() {
  _worldLocation += glm::normalize(_worldLocation) * MOVEMENT_DELTA;
  computeView();
}

void Camera::inputMoveLeft() {
  _worldLocation += glm::normalize(glm::cross(_worldLocation, _up)) * MOVEMENT_DELTA;
  computeView();
}

void Camera::inputMoveRight() {
  _worldLocation -= glm::normalize(glm::cross(_worldLocation, _up)) * MOVEMENT_DELTA;
  computeView();
}

void Camera::set_aspect_ratio(float aspect) {
    _aspectratio = aspect;
    computeProjection();
}