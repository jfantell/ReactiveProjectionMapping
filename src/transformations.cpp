// Create transformation matrices required to render the RealSense data
// The transformation matrices will be computed in the C++ program and
// sent to the vertex shader


#include "transformations.h"
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;

// Generate perspective matrix
void perspective_matrix(GLFWwindow* win){
    int actualScreenWidth, actualScreenHeight;
    float aspect; // Aspect ratio of OpenGL window
    glfwGetFramebufferSize(win, &actualScreenWidth, &actualScreenHeight);
    glViewport(0, 0, actualScreenWidth, actualScreenHeight);

    aspect = (float)actualScreenWidth/(float)actualScreenWidth;
    pMat = glm::perspective(1.0472f, aspect, 0.01f, 20.0f);
}

void print_homogenous(glm::vec4 &point,const char * identifier){
    std::cout << identifier << " " << point.x/point.w << " " << point.y/point.w << " " << point.z/point.w << endl;
}

// Checks if a matrix is a valid rotation matrix.
void model_view_matrix(){

    float cameraX, cameraY, cameraZ;
    float objectLocX, objectLocY, objectLocZ;

    //METHOD 1: INITIALIZE MV MATRIX AS FOLLOWS
    // Initialize camera and model positions
    cameraX = 0.0f; cameraY = 0.0f; cameraZ = 0.0f;
    objectLocX = 0.0f; objectLocY = 5.0f; objectLocZ = 10.0f;

    //build model and view matrix
//    vMat = translate(glm::mat4(1.0f), glm::vec3(-cameraX,-cameraY,-cameraZ));
//    mMat = translate(glm::mat4(1.0f), glm::vec3(objectLocX, objectLocY, objectLocZ));
//    mvMat = vMat * mMat;

    //Model scaling
//    glm::vec3 scale = glm::vec3(5.0f, 5.0f, 5.0f);
//    mMat = glm::scale(mMat, scale);
//    mvMat = vMat * mMat;

    //METHOD 2: GLU LOOKAT MATRIX
    glm::vec3 eye = glm::vec3(0,0,0); //EYE
    glm::vec3 at = glm::vec3(0, 0, 1); //AT
    glm::vec3 up = glm::vec3(0,-1,0); //UP
    mvMat = lookAt(eye,at,up);

    //DIAGNOSTICS
    glm::vec4 projected_point_1 =  pMat * mvMat * point_1;
    glm::vec4 projected_point_2 = pMat * mvMat * point_2;
    print_homogenous(projected_point_1,"Projected Point 1");
    print_homogenous(projected_point_2, "Projected Point 2");
    // Print projected point
//    std::cout << "Projected Point 1 " << glm::to_string(projected_point_1) << std::endl;
//    std::cout << "Projected Point 2 " << glm::to_string(projected_point_2) << std::endl;

    //INTERACTIVE TRANSLATIONS AND ROTATIONS
    //COMMENTING THIS DISABLES USER INPUT
    mvMat = glm::translate(mvMat, glm::vec3(0, 0, 0.05+(float)windowState.offset_y*.2));
    mvMat = glm::rotate(mvMat, glm::radians((float)-windowState.pitch), glm::vec3(1.0f, 0.0f, 0.0f));
    mvMat = glm::rotate(mvMat, glm::radians((float)windowState.yaw), glm::vec3(0.0f, 1.0f, 0.0f));
    mvMat = glm::translate(mvMat, glm::vec3(0, 0, -0.5f));
    cout << " Yaw " << windowState.yaw << " Pitch " << windowState.pitch << " Zoom Offset " << windowState.offset_y << endl;

}