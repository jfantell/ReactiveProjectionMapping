#ifndef SHADOWS_SHADOWS_INCLUDE_POINTLIGHT_H_
#define SHADOWS_SHADOWS_INCLUDE_POINTLIGHT_H_

#include "GL/glew.h"
#include "glm/vec3.hpp"

class PointLight {
 public:
  PointLight(GLuint shaderProgramId) {
    _shaderProgramId = shaderProgramId;
    _uniformAmbientLightStrength = glGetUniformLocation(_shaderProgramId, "ambientLightStrength");
    _uniformDiffuseLightStrength = glGetUniformLocation(_shaderProgramId, "diffuseLightStrength");
    _uniformAmbientLightColor = glGetUniformLocation(_shaderProgramId, "ambientLightColor");
    _uniformSpecularStrength = glGetUniformLocation(_shaderProgramId, "specularStrength");
    _uniformLightPosition = glGetUniformLocation(_shaderProgramId, "lightPosition");
  };
  ~PointLight() {}

  void updateShaderUniforms() {
    glUniform1f(_uniformAmbientLightStrength, _ambientStrength);
    glUniform1f(_uniformDiffuseLightStrength, _diffuseStrength);
    glUniform1f(_uniformSpecularStrength, _specularStrength);
    glUniform3f(_uniformAmbientLightColor, _ambientColor.r, _ambientColor.g, _ambientColor.b);
    glUniform3f(_uniformLightPosition, _location.x, _location.y, _location.z);
  }

  glm::vec3 getWorldLocation() { return _location; }
  void setWorldLocation(glm::vec3 worldLocation) { _location = worldLocation; }

  float getAmbientStrength() { return _ambientStrength; }
  void setAmbientStrength(float ambientStrength) { _ambientStrength = ambientStrength; }

  float getSpecularStrength() { return _specularStrength; }
  void setSpecularStrength(float specularStrength) {_specularStrength = specularStrength; }

  glm::vec3 getAmbientColor() {return _ambientColor; }
  void setAmbientColor(glm::vec3 ambientColor) { _ambientColor = ambientColor; }

  float getDiffuseStrength() { return _diffuseStrength; }
  void setDiffuseStrength(float diffuseStrength) { _diffuseStrength = diffuseStrength; }

 private:


  glm::vec3 _location = glm::vec3(1.0,1.0,0.0);
  float _ambientStrength = 0.5f;
  float _diffuseStrength = 2.0f;
  float _specularStrength = 0.3f;
  glm::vec3 _ambientColor = glm::vec3(1.0f, 1.0f, 1.0f);

  GLuint _shaderProgramId = 0;
  GLuint _uniformAmbientLightStrength = 0;
  GLuint _uniformDiffuseLightStrength = 0;
  GLuint _uniformAmbientLightColor = 0;
  GLuint _uniformSpecularStrength = 0;
  GLuint _uniformLightPosition = 0;

};

#endif //SHADOWS_SHADOWS_INCLUDE_POINTLIGHT_H_
