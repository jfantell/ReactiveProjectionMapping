#include "indexbuffer.h"
#include "GL/glew.h"

IndexBuffer::IndexBuffer() { _bufferId = -1;}

IndexBuffer::~IndexBuffer() {}

int IndexBuffer::genBuffer() {
  glGenBuffers(1, (GLuint *) &_bufferId);
  return _bufferId;
}

void IndexBuffer::bind() {
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _bufferId);
}

void IndexBuffer::unbind() {
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void IndexBuffer::setBufferPtr(void *buffer) {
  _buffer = buffer;
}

void * IndexBuffer::getBufferPtr() {
  return _buffer;
}





