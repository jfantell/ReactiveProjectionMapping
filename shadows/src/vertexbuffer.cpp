#include "vertexbuffer.h"
#include <iostream>
#include <GL/glew.h>

VertexBuffer::VertexBuffer() {
 _bufferId = -1;
}

VertexBuffer::~VertexBuffer() {
  if (_buffer != nullptr){
    //free _buffer;
  }
}

int VertexBuffer::genBuffer() {
  glGenBuffers(1, (GLuint*)&_bufferId);
  return _bufferId;
}

void VertexBuffer::bind() {
  glBindBuffer(GL_ARRAY_BUFFER, _bufferId);
}

void VertexBuffer::unbind() {
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void VertexBuffer::setBufferPtr(void * buffer) {
  _buffer = buffer;
}

void * VertexBuffer::getBufferPtr() {
  return _buffer;
}



