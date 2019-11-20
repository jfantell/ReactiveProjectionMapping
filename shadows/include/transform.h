#ifndef SHADOWS_SHADOWS_INCLUDE_TRANSFORM_H_
#define SHADOWS_SHADOWS_INCLUDE_TRANSFORM_H_

#include "glm/mat4x4.hpp"
#include "glm/vec3.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include <math.h>

class Transform {
 public:
  Transform() {
    computeModel();
  }
  ~Transform() { };

  glm::mat4 getModelMatrix() { return _model; }
  void setX(float x) { _worldPosition.x = x; computeModel(); }
  void setY(float y) { _worldPosition.y = y; computeModel(); }
  void setZ(float z) { _worldPosition.z = z; computeModel(); }

  float getX() { return _worldPosition.x; }
  float getY() { return _worldPosition.y; }
  float getZ() { return _worldPosition.z; }

  glm::vec3 getWorldPosition() { return _worldPosition; }
  void setWorldPosition(glm::vec3 worldPosition) { _worldPosition = worldPosition; computeModel(); }

  void setXRotationDegrees(float xDegrees) { _xRotationRadians = glm::radians(xDegrees); computeModel(); }
  void setYRotationDegrees(float yDegrees) { _yRotationRadians = glm::radians(yDegrees); computeModel(); }
  void setZRotationDegrees(float zDegrees) { _zRotationRadians = glm::radians(zDegrees); computeModel(); }

  void rotateXDegrees(float xDegrees) {_xRotationRadians += glm::radians(xDegrees); computeModel(); }
  void rotateYDegrees(float yDegrees) {_yRotationRadians += glm::radians(yDegrees); computeModel(); }
  void rotateZDegrees(float zDegrees) {_zRotationRadians += glm::radians(zDegrees); computeModel(); }

  void setModelScale(float modelScale) { _scale = modelScale; computeModel(); }


 private:

  void computeModel() {
    // Scale first
    glm::vec3 scale = glm::vec3(_scale, _scale, _scale);
    _model = glm::scale(glm::mat4(1.0f), scale);

    // Rotate around all three axes
    _model = glm::rotate(_model, _xRotationRadians, glm::vec3(1.0f, 0.0f, 0.0f));
    _model = glm::rotate(_model, _yRotationRadians, glm::vec3(0.0f, 1.0f, 0.0f));
    _model = glm::rotate(_model, _zRotationRadians, glm::vec3(1.0f, 0.0f, 1.0f));

    // Translate
    _model = glm::translate(_model, _worldPosition);
  }

  glm::vec3 _worldPosition = glm::vec3(0.0f, 0.0f, 0.0f);
  float _scale = 1.0f;
  float _xRotationRadians = 0.0f;
  float _yRotationRadians = 0.0f;
  float _zRotationRadians = 0.0f;

  glm::mat4 _model;


};


#endif //SHADOWS_SHADOWS_INCLUDE_TRANSFORM_H_
