#include "shader.h"
#include <fstream>
#include <vector>
#include <string.h>
#include <iostream>

// Use either GL_FRAGMENT_SHADER or GL_VERTEX_SHADER
Shader::Shader(unsigned short shadertype) {
  _shaderType = shadertype;
  memset(_shaderSource, 0x00, MAX_SHADER_SIZE);
}

int Shader::loadShaderSource(const std::string path) {
  std::ifstream shaderFile;
  shaderFile.open(path);
  std::string source;
  std::string line;
  if (shaderFile.is_open()) {
    while(getline(shaderFile, line))
    {
      source += line + "\n";
    }

    const char * source_cstr = source.c_str();
    strcpy(_shaderSource, source_cstr);
//    std::cout << _shaderSource << std::endl;
    shaderFile.close();
    return 0;
  }
  else
  {
    std::cerr << "ERROR - Could not open shaderfile: " << path << std::endl;
  }
  return -1;
}

Shader::~Shader() {
  if (_compiled) glDeleteShader(_shaderId);
}

const unsigned short Shader::getShaderType() {
  return _shaderType;
}

const char * Shader::getShaderSource() {
  return _shaderSource;
}

GLuint Shader::compile() {
  GLuint s = glCreateShader(_shaderType);
  // Avoid const issues with a stack variable.
  const char * src = getShaderSource();
  glShaderSource(s, 1, &src, NULL);
  glCompileShader(s);

  GLint isCompiled = 0;
  glGetShaderiv(s, GL_COMPILE_STATUS, &isCompiled);
  if(isCompiled == GL_FALSE)
  {
    GLint maxLength = 0;
    glGetShaderiv(s, GL_INFO_LOG_LENGTH, &maxLength);

    // The maxLength includes the NULL character
    std::vector<GLchar> errorLog(maxLength);
    glGetShaderInfoLog(s, maxLength, &maxLength, &errorLog[0]);

    // Dump the compilation error to cerr
    std::string shaderError;
      shaderError += (char*) &errorLog[0];

    std::cerr << "Shader Compilation Error in " << (_shaderType == GL_VERTEX_SHADER ? "vertex shader." : "fragment shader") << std::endl;
    std::cerr << "GLSL Error:" << std::endl;
    std::cerr << "\t" << shaderError << std::endl;

    glDeleteShader(s); // Don't leak the shader.
    exit(-1); // Exit with failure.
  }
  _shaderId = s;
  _compiled = true;
  return s;
}

