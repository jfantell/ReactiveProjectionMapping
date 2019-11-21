#include <vector>
#include <iostream>

#ifndef SHADOWS_SHADOWS_INCLUDE_VERTEXBUFFER_H_
#define SHADOWS_SHADOWS_INCLUDE_VERTEXBUFFER_H_

class VertexBuffer {
 public:
  VertexBuffer();
  ~VertexBuffer();

  int genBuffer();
  void bind();
  void unbind();
  void setBufferPtr(void * buffer);
  void * getBufferPtr();
  int getBufferId() {return _bufferId;}

 private:
  int _bufferId;
  void * _buffer = nullptr;
};

#endif //SHADOWS_SHADOWS_INCLUDE_VERTEXBUFFER_H_
