#include "vertexbuffer.h"
#include <iostream>

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

void VertexBuffer::interleave(unsigned int vertexcount) {
  unsigned char * arrayptr = (unsigned char * ) malloc(_layout.getVertexSize() * vertexcount);
  int offsets[_layout._layout.size()];
  int arrayoffset = 0;
  int elementsize = 0;

  // zero the offsets
  for (int i = 0; i < _layout._layout.size(); i++) offsets[i] = 0;

  // Loop through for every vertex, and copy the correct amount of data from each pointer.

  // For each vertex that we're supposed to have
  for (int i = 0; i < vertexcount; i++) {
    // And for every element in the layout
    for (int j = 0; j < _layout._layout.size(); j++) {
      // Get the size of the element
      elementsize = _layout._layout[j].count * _layout.getSizeOfType(_layout._layout[j].type);
      // Copy in the correct number of bytes by count * typesize
      memcpy(((unsigned char*) arrayptr) + arrayoffset, ((unsigned char *) _layout._layout[j].dataBasePTR) + offsets[j], elementsize);
      // Update the offsets.
      offsets[j] += elementsize;
      arrayoffset += elementsize;
    }
  }
  _buffer = arrayptr;
}

void VertexBuffer::setBufferPtr(void * buffer) {
  _buffer = buffer;
}

void * VertexBuffer::getBufferPtr() {
  return _buffer;
}



