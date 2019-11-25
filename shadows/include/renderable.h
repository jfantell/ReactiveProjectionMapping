#ifndef SHADOWS_SHADOWS_INCLUDE_RENDERABLE_H_
#define SHADOWS_SHADOWS_INCLUDE_RENDERABLE_H_

#include <iostream>
#include <string.h>
#include <librealsense2/rs.hpp>
#include "transform.h"
#include "camera.h"
#include "GL/glew.h"

#define numVAOs 1
#define numVBOs 5

class Renderable {
 public:

  virtual ~Renderable() {}
  virtual void setup() = 0; // MUST be implemented in base class
  virtual Transform * getTransform() {return &_transform; }
  virtual GLuint* getVBO() {return _vbo;}
  virtual GLuint* getVAO() {return _vao;}
  virtual long int get_num_vertices() {return _num_vertices; }
  virtual GLuint get_textureID() {return _texId; }
  virtual void set_description(std::string &description) {_description = description;}
  virtual void refresh(rs2::frameset &frames) = 0; // MUST be implemented in base class

 protected:
    GLuint _vao[numVAOs];
    GLuint _vbo[numVBOs];
    std::string _description;
    Transform _transform;
    long int _num_vertices;
    GLuint _texId;
};



//class Base {
// public:
//  virtual ~Base() {} // As always, you want the destructor to be virtual when virtual functions are present.
//  virtual int standard(int n) { return n; }; // Standard virtual function
//  virtual int pure(int n) = 0; // Pure virtual function
//};

#endif //SHADOWS_SHADOWS_INCLUDE_RENDERABLE_H_
