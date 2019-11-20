#ifndef SHADOWS_SHADOWS_INCLUDE_RENDERABLE_H_
#define SHADOWS_SHADOWS_INCLUDE_RENDERABLE_H_

#include "transform.h"
#include "camera.h"
#include "GL/glew.h"

class Renderable {
 public:

  virtual ~Renderable() {}
  virtual void setup() = 0; // MUST be implemented in base class
  virtual void draw() = 0;  // Must be implemented in base class
  virtual Transform * getTransform() {return &_transform; }




 protected:
  GLuint _shaderId;
  Camera * _camera;
  Transform _transform;

};



//class Base {
// public:
//  virtual ~Base() {} // As always, you want the destructor to be virtual when virtual functions are present.
//  virtual int standard(int n) { return n; }; // Standard virtual function
//  virtual int pure(int n) = 0; // Pure virtual function
//};

#endif //SHADOWS_SHADOWS_INCLUDE_RENDERABLE_H_
