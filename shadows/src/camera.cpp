#include "camera.h"
#include <iostream>
#include "GLFW/glfw3.h"
#include "glm/glm.hpp"
#include "app_state.h"

#define MOVEMENT_DELTA 0.02f
#define MOUSE_DEGREES_PER_PIXEL 0.02f


Camera::Camera(GLFWwindow * window, GLuint shaderProgramId, glm::vec3 eye, glm::vec3 at, glm::vec3 up) {
  _shaderProgramId = shaderProgramId;
  _eye = eye;
  _eyeDefault = eye;
  _at = at;
  _up = up;
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

void Camera::computeView() {
  _view = glm::lookAt(
          _eye, // Camera is at (4,3,3), in World Space
      _eye + _viewDirection, // and looks at the origin
      _up  // Head is up (set to 0,-1,0 to look upside-down)
  );
  #ifdef REALSENSE
    // BROWN ROTATION + TRANSLATION MATRIX (Comment out when not using projector)
    float rBrown[9] = {0.996507808516201, -0.03092390501794334, -0.07756223091608143, 0.02496544786410276, 0.9967455972718228, -0.07664816196218925, 0.0796800726605033, 0.07444411607157712, 0.994036799923996};
    float tBrown[3] = {105.2294283662379,-84.08153502322591,32.49814771812322};
    glm::mat4 T = glm::translate(glm::mat4(1.0f), glm::vec3(tBrown[0]/1000, tBrown[1]/-1000,tBrown[2]/1000)); // scaled and y,z axes are switched
    glm::mat4 R = glm::mat4(rBrown[0], rBrown[1], rBrown[2], 0, rBrown[3], rBrown[4], rBrown[5], 0, rBrown[6], rBrown[7], rBrown[8], 0, 0,0,0,1);
    _view = R * _view;
    _view = T * _view;

  //  _view = glm::rotate(_view, glm::radians((float)windowState.pitch), glm::vec3(0.0f, 1.0f, 0.0f)); //y
    _view = glm::rotate(_view, glm::radians(2.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    _view = glm::rotate(_view, glm::radians(2.0f), glm::vec3(0.0f, 1.0f, 0.0f)); //y
  #endif


}

void Camera::computeProjection() {
#ifdef REALSENSE
    //METHOD 1: Projection based on Brown Code (FOR PROJECTION MAPPING)
    float near = 0.01;
    float far = 10;
    float A = near + far;
    float B = near * far;
    float width_ = 960.0;
    float height_ = 560.0;
    float kBrown[9] = {320.913404297975, 0, 464.073296207169, 0, 317.2041738539567, 290.5717543103951, 0, 0, 1};

    glm::mat4 Persp = glm::mat4(kBrown[0]*2.8, kBrown[1], -1*kBrown[2], 0, kBrown[3], -1*kBrown[4]*2.8, -1*kBrown[5], 0, 0, 0, A,B, kBrown[6], kBrown[7], -1*kBrown[8], 0);
    Persp = glm::transpose(Persp);
    glm::mat4 NDC = glm::ortho(0.0f, width_, height_, 0.0f, near, far);
    _projection = NDC * Persp;
  #else
    //METHOD 2: Default OpenGL Projection
    _projection = glm::perspective(windowState.get_fov(), windowState.get_aspect_ratio(), windowState.get_z_near(), windowState.get_z_far());
  #endif
}

void Camera::updateShaderUniforms() {
  glUniform3f(_uniformViewPosition, _eye.x, _eye.y, _eye.z);
}


void Camera::inputMoveUp() {
    _eye += glm::normalize(_viewDirection) * MOVEMENT_DELTA;
  computeView();
}

void Camera::inputMoveDown() {
    _eye -= glm::normalize(_viewDirection) * MOVEMENT_DELTA;
  computeView();
}

void Camera::inputMoveLeft() {
    _eye -= glm::normalize(glm::cross(_viewDirection, _up)) * MOVEMENT_DELTA;
  computeView();
}

void Camera::inputMoveRight() {
    _eye += glm::normalize(glm::cross(_viewDirection, _up)) * MOVEMENT_DELTA;
  computeView();
}

void Camera::reset_aspect_ratio() {
    computeProjection();

}

void Camera::zoom(float amount) {
    _eye += amount * glm::normalize(_viewDirection) * MOVEMENT_DELTA;
  computeView();
}

void Camera::strafe(float amount) {
    _eye += -amount * glm::normalize(glm::cross(_viewDirection, _up)) * MOVEMENT_DELTA;
}


void Camera::updateMouse(const glm::vec2 &newMousePosition) {

  glm::vec2 mouseDelta = glm::vec2(newMousePosition.x - windowState.last_x, newMousePosition.y - windowState.last_y);
  mouseDelta *= MOUSE_DEGREES_PER_PIXEL;

  // Do the yaw. Full 360 degree rotation is allowed.
  _viewDirection = glm::mat3(glm::rotate(-mouseDelta.x, _up)) * _viewDirection;
  windowState.last_x = newMousePosition.x;
  windowState.last_y = newMousePosition.y;
  windowState.yaw += -mouseDelta.x;
  windowState.pitch += -mouseDelta.y;

  std::cout << "Yaw " << windowState.yaw << " Pitch " << windowState.pitch << std::endl;

  glm::vec3 across = glm::normalize(glm::cross(_viewDirection, _up));

  // Do the pitch rotation. we're going to clip the rotation to an angle range.
  _viewDirection = glm::mat3(glm::rotate(-mouseDelta.y, across)) * _viewDirection;
  std::cout << "viewDirection: " << _viewDirection.x << " " << _viewDirection.y << " " << _viewDirection.z << std::endl;

  float angle = glm::acos(glm::dot(_viewDirection, _up));
  glm::vec3 cross = glm::cross(_viewDirection, _up);
  if(glm::dot(cross, across) < 0) angle = -angle;

  std::cout << "angle: " << angle << std::endl;

  if (angle < glm::radians(45.0)) std::cout << "looking too far up." << std::endl;
  if (angle > glm::radians(137.0)) std::cout << "looking too far down. " << std::endl;

  //Rotate and Translate View Matrix
  computeView();
}

void Camera::restoreDefaultEyeLocation(){
    _viewDirection = _viewDirectionDefault;
    _eye = _eyeDefault;
    computeView();
}