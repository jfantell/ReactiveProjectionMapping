// Utility functions for error checking
// Modified from Computer Graphics Programming in OpenGL with C++ by V. Scott Gordon
// and John L. Clevenger

#ifndef utils_hpp
#define utils_hpp

#include <GLFW/glfw3.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

bool checkOpenGLError();
void printProgramLog(int prog);
void printShaderLog(GLuint shader);
std::string readShaderSource(const char *filePath);
GLuint createShaderProgram(const char *vertShaderFile, const char *fragShaderFile);
GLuint loadTexture(const char *texImagePath);
void loadTexture(GLuint textureRef, cv::Mat &image_frame);

#endif /* utils_hpp */
