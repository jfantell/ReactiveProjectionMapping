#ifndef SHADOWS_SHADOWS_VERTEXBUFFERLAYOUT_H_
#define SHADOWS_SHADOWS_VERTEXBUFFERLAYOUT_H_

#include <vector>
#include "GL/glew.h"

// TODO incorporate storing pointers to the data sources here instead of _pointers
struct VertexBufferElement {
  unsigned int type;
  unsigned int count;
};

class VertexBufferLayout {
 public:
  VertexBufferLayout();
  unsigned int getVertexSize();

  unsigned int getSizeOfType(unsigned int type);

  template<typename T>
  void push(unsigned int count) {}

  template<>
  void push<float>(unsigned int count) {
    VertexBufferElement e;
    e.type = GL_FLOAT;
    e.count = count;
    _layout.push_back(e);
  }

  std::vector<VertexBufferElement> _layout;

};

#endif //SHADOWS_SHADOWS_VERTEXBUFFERLAYOUT_H_
