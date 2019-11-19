#ifndef SHADOWS_SHADOWS_INCLUDE_SHADER_H_
#define SHADOWS_SHADOWS_INCLUDE_SHADER_H_

#define MAX_SHADER_SIZE 1500

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
    GLuint getShaderId() { return _shaderId; }

   private:
    unsigned short _shaderType;
    char _shaderSource[MAX_SHADER_SIZE];
    GLuint _shaderId;
    bool _compiled = false;

  };

  static GLuint buildShaderProgram(Shader &fs, Shader &vs) {
    GLuint program = glCreateProgram();
    glAttachShader(program, fs.getShaderId());
    glAttachShader(program, vs.getShaderId());
    glLinkProgram(program);
    glUseProgram(program);
    return program;
  }
#endif //SHADOWS_SHADOWS_INCLUDE_SHADER_H_
