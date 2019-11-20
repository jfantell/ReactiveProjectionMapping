#ifndef SHADOWS_SHADOWS_INCLUDE_R_r_LightMarker_H_
#define SHADOWS_SHADOWS_INCLUDE_R_r_LightMarker_H_

#include "renderable.h"
#include "pointlight.h"

class r_LightMarker : public Renderable {
 public:
  r_LightMarker(GLuint shaderProgramId, Camera * c, PointLight * p);
  ~r_LightMarker() {}

  void setup();
  void draw();

 private:

  void updatePosition();

  Renderable * _r;
  PointLight * _p;
  Camera * _c;
  glm::vec3 _worldLocation = glm::vec3(0,0,0);


  GLuint _shaderProgramId = 0;
};

#endif //SHADOWS_SHADOWS_INCLUDE_R_r_LightMarker_H_
