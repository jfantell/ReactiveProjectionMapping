#include "camera.h"
#include <iostream>
#include "GLFW/glfw3.h"
#include "glm/glm.hpp"
#include "app_state.h"

#define MOVEMENT_DELTA 0.02f
#define MOUSE_DEGREES_PER_PIXEL .02f;


Camera::Camera(GLFWwindow * window, GLuint shaderProgramId, float fov_degrees, float aspectratio, glm::vec3 worldLocation, glm::vec3 lookAt) {
  _shaderProgramId = shaderProgramId;
  _fovRadians = glm::radians(fov_degrees);
  _aspectratio = aspectratio;
  _worldLocation = worldLocation;
  _worldLocationDefault = worldLocation;
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
      _worldLocation + _viewDirection, // and looks at the origin
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
  _worldLocation += glm::normalize(_viewDirection) * MOVEMENT_DELTA;
  computeView();
}

void Camera::inputMoveDown() {
  _worldLocation -= glm::normalize(_viewDirection) * MOVEMENT_DELTA;
  computeView();
}

void Camera::inputMoveLeft() {
  _worldLocation -= glm::normalize(glm::cross(_viewDirection, _up)) * MOVEMENT_DELTA;
  computeView();
}

void Camera::inputMoveRight() {
  _worldLocation += glm::normalize(glm::cross(_viewDirection, _up)) * MOVEMENT_DELTA;
  computeView();
}

void Camera::set_aspect_ratio(float aspect) {
    _aspectratio = aspect;
    computeProjection();
}

void Camera::zoom(float amount) {
  _worldLocation += amount * glm::normalize(_viewDirection) * MOVEMENT_DELTA;
  computeView();
}


void Camera::updateMouse(const glm::vec2 &newMousePosition) {

  glm::vec2 mouseDelta = glm::vec2(newMousePosition.x - windowState.last_x, newMousePosition.y - windowState.last_y);
  mouseDelta *= MOUSE_DEGREES_PER_PIXEL;

  // Do the yaw. Full 360 degree rotation is allowed.
  _viewDirection = glm::mat3(glm::rotate(-mouseDelta.x, _up)) * _viewDirection;
  windowState.last_x = newMousePosition.x;
  windowState.last_y = newMousePosition.y;

  glm::vec3 across = glm::normalize(glm::cross(_viewDirection, _up));

  // Do the pitch rotation. we're going to clip the rotation to an angle range.

  _viewDirection = glm::mat3(glm::rotate(-mouseDelta.y, across)) * _viewDirection;
  std::cout << "viewDirection: " << _viewDirection.x << " " << _viewDirection.y << "" << _viewDirection.z << std::endl;

  float angle = glm::acos(glm::dot(_viewDirection, _up));
  glm::vec3 cross = glm::cross(_viewDirection, _up);
  if(glm::dot(cross, across) < 0) angle = -angle;

  std::cout << "angle: " << angle << std::endl;

  if (angle < glm::radians(45.0)) std::cout << "looking too far up." << std::endl;
  if (angle > glm::radians(137.0)) std::cout << "looking too far down. " << std::endl;

  computeView();
}

void Camera::restoreDefaultWorldLocation(){
    _worldLocation = _worldLocationDefault;
    computeView();
}