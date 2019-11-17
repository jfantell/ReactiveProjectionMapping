#ifndef SHADOWS_SHADOWS_INCLUDE_INDEXBUFFER_H_
#define SHADOWS_SHADOWS_INCLUDE_INDEXBUFFER_H_

#include <vector>
#include "GL/glew.h"

class IndexBuffer {
 public:
  IndexBuffer();
  ~IndexBuffer();

  int genBuffer();
  void bind();
  void unbind();
  void setBufferPtr(void * buffer);
  void * getBufferPtr();
  int getBufferId() { return _bufferId;}

 private:
  int _bufferId;
  void * _buffer = nullptr;

};

#endif //SHADOWS_SHADOWS_INCLUDE_INDEXBUFFER_H_
