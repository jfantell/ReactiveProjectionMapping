#include "r_lightmarker.h"
#include "stb_image.h"
#include "r_rabbit.h"
#include "r_objmodel.h"
#include <iostream>



r_LightMarker::r_LightMarker(GLuint shaderProgramId, Camera * c, PointLight * p) {
  _p = p;
  _c = c;
  _shaderProgramId = shaderProgramId;
  _r = new r_OBJModel(_shaderProgramId, _c, "../res/cube.obj", "../res/blanktex.png");

}

void r_LightMarker::setup() {
  _r->setup();
  _r->getTransform()->setModelScale(0.7f);
  updatePosition();
}

void r_LightMarker::draw() {
  updatePosition();
  _r->draw();
}

void r_LightMarker::updatePosition() {
  glm::vec3 lightPosition = _p->getWorldLocation();
  _r->getTransform()->setWorldPosition(lightPosition);
}
