#ifndef SHADOWS_SHADOWS_INCLUDE_SHADER_H_
#define SHADOWS_SHADOWS_INCLUDE_SHADER_H_

#define MAX_SHADER_SIZE 1000

#include <iostream>
#include <stdlib.h>
#include "GL/glew.h"

class Shader {
 public:
  Shader(const unsigned short shadertype);
  ~Shader();
  int loadShaderSource(const std::string path);
  const unsigned short getShaderType();
  const char * getShaderSource();
  GLuint compile();

 private:
  unsigned short _shaderType;
  char _shaderSource[MAX_SHADER_SIZE];
  GLuint _shaderId;
  bool _compiled = false;

};

#endif //SHADOWS_SHADOWS_INCLUDE_SHADER_H_
