#include "camera.h"


Camera::Camera(float fov_degrees, float aspectratio, glm::vec3 worldLocation, glm::vec3 lookAt) {
  _fovRadians = glm::radians(fov_degrees);
  _aspectratio = aspectratio;
  _worldLocation = worldLocation;
  _lookAt = lookAt;

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
}

glm::vec3 Camera::getWorldLocation() {
  return _worldLocation;
}

void Camera::computeView() {
  _view = glm::lookAt(
      _worldLocation, // Camera is at (4,3,3), in World Space
      _lookAt, // and looks at the origin
      glm::vec3(0,1,0)  // Head is up (set to 0,-1,0 to look upside-down)
  );
}

void Camera::computeProjection() {
  // Projection matrix : 45° Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
  _projection = glm::perspective(_fovRadians, _aspectratio, 0.1f, 100.0f);

}