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

    float near = 0.01;
    float far = 10;
    float A = near + far;
    float B = near * far;
    float width_ = 960.0;
    float height_ = 560.0;

    float kBrown[9] = {320.913404297975, 0, 464.073296207169, 0, 317.2041738539567, 290.5717543103951, 0, 0, 1};

    //METHOD #1: USING GIVEN K MATRIX AND NDC MATRIX TO GENERATE PERSPECTIVE PROJECTION
    // AS DEMONSTRATED IN ONLINE TUTORIAL: http://ksimek.github.io/2013/06/03/calibrated_cameras_in_opengl/
   // glm::mat4 Persp = glm::mat4(kBrown[0], kBrown[1], -0.5*kBrown[2], 0, kBrown[3], -1*kBrown[4], -0.5*kBrown[5], 0, 0, 0, A,B, kBrown[6], kBrown[7], -1*kBrown[8], 0);
    glm::mat4 Persp = glm::mat4(kBrown[0]*3.1, kBrown[1], -1*kBrown[2], 0, kBrown[3], -1*kBrown[4]*3.1, -1*kBrown[5], 0, 0, 0, A,B, kBrown[6], kBrown[7], -1*kBrown[8], 0);

    //CONVERT FROM ROW MAJOR ORDER TO COLUMN MAJOR ORDER
    Persp = glm::transpose(Persp);


    glm::mat4 NDC = glm::ortho(0.0f, 960.0f, 560.0f, 0.0f, near, far);
    pMat = NDC * Persp;
    //PRINT PMAT MATRIX
    cout << glm::to_string(pMat) << endl;

    //IGNORE
    //    glm::mat4 Persp = glm::mat4(0.97, 0, 0, 0,
//                      0, 1.73, 0, 0,
//                      0, 0, A, B,
//                      0, 0, -1, 0);


    //METHOD 2: DEFAULT OPENGL PERSPECTIVE MATRIX (FOR DEBUGGING)
    glm::mat4 pMat2 = glm::perspective(1.0472f, aspect, 0.01f, 10.0f);
    //pMat = glm::perspective(1.0472f, aspect, 0.01f, 10.0f);
    cout << glm::to_string(pMat2) << endl;
}

// Checks if a matrix is a valid rotation matrix.
void model_view_matrix(){

    float cameraX, cameraY, cameraZ;
    float objectLocX, objectLocY, objectLocZ;



    //METHOD 1: INITIALIZE MV MATRIX AS FOLLOWS
    // Initialize camera and model positions
    cameraX = 0.0f; cameraY = 0.0f; cameraZ = 0.0f;
    objectLocX = 0.0f; objectLocY = 0.0f; objectLocZ = 1.0f;
    //build view matrix
    vMat = translate(glm::mat4(1.0f), glm::vec3(-cameraX,-cameraY,-cameraZ));
    // rotate view matrix (if needed)
//    vMat = rotate(vMat,radians(180.0f),vec3(1,0,0));
    //build model matrix
    mMat = translate(glm::mat4(1.0f), glm::vec3(objectLocX, objectLocY, objectLocZ));
    //build model view matrix
    mvMat = vMat * mMat;


    //Model scaling
//    glm::vec3 scale = glm::vec3(5.0f, 5.0f, 5.0f);
//    mMat = glm::scale(mMat, scale);
//    mvMat = vMat * mMat;


    //METHOD 2: GLU LOOKAT MATRIX
    glm::vec3 eye = glm::vec3(0,0,0); //EYE
    glm::vec3 at = glm::vec3(0, 0, .01); //AT
    glm::vec3 up = glm::vec3(0,-1,0); //UP
    mvMat = lookAt(eye,at,up);

    //DIAGNOSTICS
    glm::vec4 projected_point_1 =  mvMat * point_1;
    glm::vec4 projected_point_2 = mvMat * point_2;
    // Print projected point
//    std::cout << "Projected Point 1 " << glm::to_string(projected_point_1) << std::endl;
//    std::cout << "Projected Point 2 " << glm::to_string(projected_point_2) << std::endl;


     //PROJECTOR EXTRINSICS: TRANSLATION AND ROTATION MATRICES
     //ROTATION MATRIX MUST BE TRANSPOSED TO ENSURE COLUMN MAJOR ORDER
    //float rBrown[9] = {0.992, -0.041, -0.114, 0.026, 0.991, -0.123, 0.118, 0.119, 0.985};
  //  float rBrown2[3][3] = {{0.992, -0.041, -0.114}, {0.026, 0.991, -0.123}, {0.118, 0.119, 0.985}};
    float rBrown[9] = {0.9993056180948953, -0.03696037875994081, -0.004712965722452996, 0.03671444479747512, 0.998333504916434, -0.04452260666764154, 0.006350683994063414, 0.04431865705535405, 0.998997259981036};
    //float tBrown[3] = {106.406, -69.097, 96.486};
    float tBrown[3] = {119.5728106196159, -53.34722888908106, 174.9022427120458};


    //mvMat = rotate(mvMat, 2*-0.006f, glm::vec3(0,1,0));

    glm::mat4 T = glm::translate(glm::mat4(1.0f), glm::vec3(tBrown[0]/1000, tBrown[1]/-1000,tBrown[2]/1000)); // scaled and y,z axises are switched
    //glm::mat4 T2 = transpose(glm::mat4(tBrown[0]/-1000,0,0,0,0, tBrown[1]/-1000,0,0,0,0,tBrown[2]/1000,0,0,0,0,0));
    // for the rotation matrix y and z need to be switched

    glm::mat4 R = glm::mat4(rBrown[0], rBrown[1], rBrown[2], 0, rBrown[3], rBrown[4], rBrown[5], 0, rBrown[6], rBrown[7], rBrown[8], 0, 0,0,0,1);
  //  glm::mat4 R = glm::mat4(rBrown[0], rBrown[1], -1*rBrown[2], 0, rBrown[3], rBrown[4], -1*rBrown[5], 0, -1*rBrown[6], -1*rBrown[7], rBrown[8], 0, 0,0,0,1);
   // R = glm::transpose(R);
//    glm::mat4 yFlip = glm::mat4(-1,0,0,0,0,1,0,0,0,0,-1,0,0,0,0,1);
//    glm::mat4 zFlip = glm::mat4(-1,0,0,0,0,1,0,0,0,0,-1,0,0,0,0,1);
//    glm::mat4 Ri = glm::inverse(R);
    mvMat = R * mvMat;
    mvMat = T * mvMat;

    //mvMat = T2;
    //mvMat = yFlip * zFlip* Ri* mvMat;
    //mvMat = Ri* T * mvMat;

    //INTERACTIVE TRANSLATIONS AND ROTATIONS
    //COMMENTING THIS DISABLES USER INPUT
    mvMat = glm::translate(mvMat, glm::vec3(0, 0, 0.05+(float)windowState.offset_y*.5));
    mvMat = glm::rotate(mvMat, glm::radians((float)windowState.pitch), glm::vec3(1.0f, 0.0f, 0.0f));
    mvMat = glm::rotate(mvMat, glm::radians((float)windowState.yaw), glm::vec3(0.0f, 1.0f, 0.0f));
    mvMat = glm::translate(mvMat, glm::vec3(0, 0, -0.5f));
    cout << " Yaw " << windowState.yaw << " Pitch " << windowState.pitch << endl;

}