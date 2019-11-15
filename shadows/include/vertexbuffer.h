#include "vertexbufferlayout.h"
#include <vector>

#ifndef SHADOWS_SHADOWS_INCLUDE_VERTEXBUFFER_H_
#define SHADOWS_SHADOWS_INCLUDE_VERTEXBUFFER_H_

class VertexBuffer {
 public:
  VertexBuffer();
  ~VertexBuffer();

  int genBuffer();
  void bind();
  void interleave(unsigned int vertexcount);
  void setBufferPtr(void * buffer);
  void * getBufferPtr();
  int getBufferId() {return _bufferId;}
  VertexBufferLayout * getLayout() {return &_layout;}
  
 private:
  VertexBufferLayout _layout;
  int _bufferId;
  void * _buffer = nullptr;
};

#endif //SHADOWS_SHADOWS_INCLUDE_VERTEXBUFFER_H_
