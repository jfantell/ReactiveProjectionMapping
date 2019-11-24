#ifndef SHADOWS_SHADOWS_INCLUDE_R_FLOOR_H_
#define SHADOWS_SHADOWS_INCLUDE_R_FLOOR_H_

#include "renderable.h"
#include "stb_image.h"
#include <iostream>

class r_Floor : public Renderable {
 public:
  r_Floor(GLuint shaderProgramId, Camera * camera);
  ~r_Floor() {}; // Overrides ~Renderable()

  void setup(); // Overrides Renderable::setup()
  void draw(); // Overrides Renderable::draw()

 private:

  float vertices_normals[3 * 2 * 8] = {
      1.0f, -1.0f, 1.0f, // first vertex
      0.0f, 1.0f, 0.0f, // first normal
      1.0f, 1.0f,          // texcoord
      1.0f, -1.0f,-1.0f,
      0.0f, 1.0f, 0.0f,
      1.0f, -1.0f,
      -1.0f, -1.0f,-1.0f,
      0.0f, 1.0f, 0.0f,
      0.0f, 0.0f,
      1.0f, -1.0f, 1.0f,
      0.0f, 1.0f, 0.0f,
      1.0f, 1.0f,
      -1.0f, -1.0f,-1.0f,
      0.0f, 1.0f, 0.0f,
      0.0f, 0.0f,
      -1.0f, -1.0f, 1.0f,
      0.0f, 1.0f, 0.0f,
      1.0f, 0.0f
  };



  GLuint _vboId = 0;
  GLuint _vaoId = 0;
  GLuint _texId = 0;
  GLuint _shaderMVPId = 0;
  GLuint _shaderModelId = 0;
  GLuint _shaderUseTexId = 0;

};

#endif //SHADOWS_SHADOWS_INCLUDE_R_FLOOR_H_
