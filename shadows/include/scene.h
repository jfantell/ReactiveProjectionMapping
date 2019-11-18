#ifndef SHADOWS_SHADOWS_INCLUDE_GLM_SCENE_H_
#define SHADOWS_SHADOWS_INCLUDE_GLM_SCENE_H_

#include "glm/vec3.hpp"
#include "glm/mat4x4.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "GL/glew.h"

class Floor {

 public:
  Floor(float scale);
  ~Floor();

  void draw();
  glm::mat4 getModelMatrix() {return _model; } // currently unused. will be in the future!

 private:
  void bindVao();

  float vertices_normals[3 * 2 * 6] = {
      1.0f, -1.0f, 1.0f, // first vertex
      0.0f, -1.0f, 0.0f,    // first normal
      1.0f, -1.0f,-1.0f,
      0.0f, -1.0f, 0.0f,
      -1.0f, -1.0f,-1.0f,
      0.0f, -1.0f, 0.0f,
      1.0f, -1.0f, 1.0f,
      0.0f, -1.0f, 0.0f,
      -1.0f, -1.0f,-1.0f,
      0.0f, -1.0f, 0.0f,
      -1.0f, -1.0f, 1.0f,
  };

  GLuint _vboId = 0;
  GLuint _vaoId = 0;
  glm::mat4 _model;

};


#endif //SHADOWS_SHADOWS_INCLUDE_GLM_SCENE_H_
