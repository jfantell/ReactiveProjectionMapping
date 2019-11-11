#include "vertexbufferlayout.h"
#include "GL/glew.h"
#include <iostream>

VertexBufferLayout::VertexBufferLayout() { }


unsigned int VertexBufferLayout::getVertexSize() {
  unsigned int size = 0;
  for (int i = 0; i < _layout.size(); i++) {
    size += getSizeOfType(_layout[i].type) * _layout[i].count;
  }
  return size;
}

unsigned int VertexBufferLayout::getSizeOfType(unsigned int type) {
  if (type == GL_FLOAT) return sizeof(float);
}
