#ifndef SHADOWS_SHADOWS_INCLUDE_R_OBJMODEL_H_
#define SHADOWS_SHADOWS_INCLUDE_R_OBJMODEL_H_

#include "renderable.h"
#include "mesh.h"
#include "vertexbuffer.h"
#include "indexbuffer.h"
#include <iostream>

class r_ObjModel : public Renderable {
 public:
  r_ObjModel(GLuint shaderProgramId, Camera * camera, std::string objpath, std::string texpath);
  ~r_ObjModel() {}; // Overrides ~Renderable()

  void setup(); // Overrides Renderable::setup();
  void draw(); // Overrides Renderable::draw()

 private:
  GLuint _shaderMVPId = 0;
  GLuint _shaderModelId = 0;
  GLuint _vaoId = 0;
  GLuint _texId = 0;
  std::string _objpath = "";
  std::string _texpath = "";
  Mesh _mesh;
  VertexBuffer _vb;
  IndexBuffer _ib;
};
#endif //SHADOWS_SHADOWS_INCLUDE_R_RABBIT_H_


