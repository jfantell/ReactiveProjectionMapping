// Utility functions for error checking and File I/O
// Modified from Computer Graphics Programming in OpenGL with C++ by V. Scott Gordon
// and John L. Clevenger

#include <iostream>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <librealsense2/rs.hpp>
#include <SOIL2/SOIL2.h>
#include <opencv2/opencv.hpp>

using namespace std;

void printShaderLog(GLuint shader){
    int len = 0;
    int chWrittn = 0;
    char *log;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &len);
    if(len>0){
        log = (char*)malloc(len);
        glGetShaderInfoLog(shader,len,&chWrittn, log);
        cout << "Shader Info Log: "<< log << endl;
        free(log);
    }
}

void printProgramLog(int prog){
    int len = 0;
    int chWrittn = 0;
    char *log;
    glGetProgramiv(prog, GL_INFO_LOG_LENGTH, &len);
    if(len > 0){
        log = (char*)malloc(len);
        glGetProgramInfoLog(prog,len,&chWrittn,log);
        cout << "Program Info Log: "<< log << endl;
        free(log);
    }
}

bool checkOpenGLError(){
    bool foundError = false;
    int glErr = glGetError();
    const GLubyte* glErrString = glewGetErrorString(glErr);
    while(glErr != GL_NO_ERROR){
        foundError = true;
        glErr = glGetError();
    }
    return foundError;
}
string readShaderSource(const char *filePath){
    ifstream file(filePath);
    cout << filePath <<endl;
//    if(file.is_open()){
//        cout<< "True"<<endl;
//    }
    string content;
    string temp;
    while(std::getline(file, temp)) {
        content.append(temp + "\n");
    }
//    cout << "Content " << contentsss << endl;
    return content;
}

GLuint createShaderProgram(const char *vertShaderFile, const char *fragShaderFile){
    GLint vertCompiled;
    GLint fragCompiled;
    GLint linked;
    
    string vertShaderStr = readShaderSource(vertShaderFile);
    string fShaderStr = readShaderSource(fragShaderFile);
    const char *vertShaderSource = vertShaderStr.c_str();
    const char *fragShaderSource = fShaderStr.c_str();
    GLuint vShader = glCreateShader(GL_VERTEX_SHADER);
    GLuint fShader = glCreateShader(GL_FRAGMENT_SHADER);
    
    glShaderSource(vShader, 1, &vertShaderSource,NULL);
    glShaderSource(fShader, 1, &fragShaderSource,NULL);
    
    glCompileShader(vShader);
    checkOpenGLError();
    glGetShaderiv(vShader,GL_COMPILE_STATUS,&vertCompiled);
    if(vertCompiled != 1){
        cout << "vertex compilation failed" << endl;
        printShaderLog(vShader);
    }
    
    glCompileShader(fShader);
    checkOpenGLError();
    glGetShaderiv(fShader,GL_COMPILE_STATUS,&fragCompiled);
    if(fragCompiled != 1){
        cout << "fragment compilation failed" << endl;
        printShaderLog(fShader);
    }
    
    GLuint vfProgram = glCreateProgram();
    glAttachShader(vfProgram,vShader);
    glAttachShader(vfProgram,fShader);
    glLinkProgram(vfProgram);
    checkOpenGLError();
    glGetShaderiv(vfProgram,GL_LINK_STATUS,&linked);
    if(linked != 1){
        cout << "linking failed " << endl;
        printProgramLog(vfProgram);
    }
    return vfProgram;
}


GLuint loadTexture(const char *texImagePath)
{   GLuint textureRef;
    textureRef = SOIL_load_OGL_texture(texImagePath, SOIL_LOAD_AUTO, SOIL_CREATE_NEW_ID, SOIL_FLAG_INVERT_Y);
    if (textureRef == 0) cout << "didnt find texture file " << texImagePath << endl;
    // ----- mipmap/anisotropic section
    glBindTexture(GL_TEXTURE_2D, textureRef);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glGenerateMipmap(GL_TEXTURE_2D);
    if (glewIsSupported("GL_EXT_texture_filter_anisotropic")) {
        GLfloat anisoset = 0.0f;
        glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &anisoset);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, anisoset);
    }
    // ----- end of mipmap/anisotropic section
    return textureRef;
}

//Source: https://github.com/IntelRealSense/librealsense/blob/master/examples/example.hpp
void loadTexture(GLuint textureRef, cv::Mat &image_frame)
{
    GLenum err = glGetError();

    glBindTexture(GL_TEXTURE_2D, textureRef);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image_frame.size().width, image_frame.size().height, 0, GL_RGB, GL_UNSIGNED_BYTE, image_frame.data);

      // ----- end of mipmap/anisotropic section
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);

}

//void loadTexture(GLuint textureRef, rs2::video_frame& frame)
//{
////    cout << textureRef << endl;
////    if(textureRef != 0){
////        cout << "what" << endl;
////        glGenTextures(1, &textureRef);
////    }
//    GLenum err = glGetError();
//
//auto format = frame.get_profile().format();
//auto width = frame.get_width();
//auto height = frame.get_height();
//
//    glBindTexture(GL_TEXTURE_2D, textureRef);
//    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, frame.size().width, image_frame.size().height, 0, GL_RGB, GL_UNSIGNED_BYTE, image_frame.data);
//    switch (format)
//    {
//        case RS2_FORMAT_RGB8:
//            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, frame.get_data());
////            break;
//        case RS2_FORMAT_RGBA8:
//            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, frame.get_data());
//            break;
//        case RS2_FORMAT_Y8:
//            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, frame.get_data());
//            break;
//        case RS2_FORMAT_Y10BPACK:
//            glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, width, height, 0, GL_LUMINANCE, GL_UNSIGNED_SHORT, frame.get_data());
////            break;
//        default:
//            throw std::runtime_error("The requested format is not supported by this demo!");
//    }
//
////    // ----- end of mipmap/anisotropic section
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
//    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
//
//}