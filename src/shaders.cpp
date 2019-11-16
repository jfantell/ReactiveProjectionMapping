//
// This file includes the logic necessary to:
//  - Create a point cloud from a RealSense depth frame
//  - Map texture coordinates to the point cloud
//  - Extract object and texture coordinates from the point cloud
//  - Connect application data to the vertex and fragment shaders
//

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"

#include "app_state.h"
#include <glm/ext.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "utils.hpp"
#include "transformations.h"
#include <opencv2/opencv.hpp>
#include <frame_processing.h>
#include <iostream>
#include "pcl_utilities.h"
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>

using namespace std;
using namespace glm;

#define numVAOs 1
#define numVBOs 2
#define maxPointsX3 2800000

GLuint renderingProgram; // Vertex and Fragment Shader Program
GLuint vao[numVAOs]; // Array to store Vertex Attribute Objects References
GLuint vbo[numVBOs]; //Array to store Vertex Attribute Buffer References
GLuint textureRef = 0; // Reference to texture address in memory
GLuint mvLoc, projLoc; // Connect C++ program to Shaders
GLsizei maxPoints; //The number of points to be drawn
std::vector<float> pointCoordinatesVector, textureCoordinatesVector; // vertex and texture coordinates

rs2::pointcloud pc; // Point cloud object
rs2::points points; // RealSense points object

long int max_ = -100000;
long int min_ = 100000;


glm::mat4 mvMat, vMat, mMat, pMat; //Define global variables
window_state windowState;
vec4 point_1, point_2;

void updatePoints(const rs2::vertex *point_coords, const rs2::texture_coordinate *text_coords){
    for (int i=0; i < points.size(); i++)
    {
        if(point_coords[i].z){
            if(point_coords[i].z > max_){
                max_ = point_coords[i].z;
                point_1 = vec4(point_coords[i].x, point_coords[i].y, point_coords[i].z, 1);
            }
            if(point_coords[i].z < min_){
                min_ = point_coords[i].z;
                point_2 = vec4(point_coords[i].x, point_coords[i].y, point_coords[i].z, 1);
            }
            pointCoordinatesVector.push_back(point_coords[i].x);
            pointCoordinatesVector.push_back(point_coords[i].y);
            pointCoordinatesVector.push_back(point_coords[i].z);
            textureCoordinatesVector.push_back(text_coords[i].u);
            textureCoordinatesVector.push_back(text_coords[i].v);
        }
    }

    maxPoints = (GLsizei)points.size();
    glBindBuffer(GL_ARRAY_BUFFER,vbo[0]);
    glBufferSubData(GL_ARRAY_BUFFER,0,(long int)pointCoordinatesVector.size()*sizeof(float),pointCoordinatesVector.data());
    glBindBuffer(GL_ARRAY_BUFFER,vbo[1]);
    glBufferSubData(GL_ARRAY_BUFFER,0,(long int)textureCoordinatesVector.size()*sizeof(float),textureCoordinatesVector.data());

}

void updateFrame(rs2::frameset &frames){
    // Vectors store memory on the heap
    // this memory needs to be erased before
    // a new frame can be drawn
    if(!pointCoordinatesVector.empty()){
        pointCoordinatesVector.clear();
    }
    if(!textureCoordinatesVector.empty()){
        textureCoordinatesVector.clear();
    }

    // Wait for the next set of frames from the RealSense Camera
    auto color = frames.get_color_frame();

    // Tell pointcloud object to map to this color frame
    pc.map_to(color);

    auto depth = frames.get_depth_frame();

    // Generate the pointcloud and texture mappings
    points = pc.calculate(depth);

    // Convert RealSense point cloud to PCL point cloud
    auto pcl_points = points_to_pcl(points);

    // Create mesh from point cloud
    auto pcl_mesh_points = create_mesh(pcl_points);

    cv::Mat image_frame;

    // Use images from realsense as textures
    if(textureMode != Image){
        if(textureMode == Color){
            image_frame = frame_to_mat(color);
        }
        else if(textureMode == Depth){
            rs2::colorizer color_map;
            rs2::frame depth_color = depth.apply_filter(color_map);
            image_frame = frame_to_mat(depth_color);
        }
        cv::Mat image_frame_processed = process_frame_image(image_frame);
        loadTexture(textureRef, image_frame_processed);
    }

    const rs2::vertex* point_coords = points.get_vertices();
    const rs2::texture_coordinate* text_coords = points.get_texture_coordinates();



    //Update points and texture coordinates
    updatePoints(point_coords, text_coords);

}

void init(GLFWwindow* window, const char * vertShaderFile, const char * fragShderFile, int textureMode_, const char * textureImage){
    // Create shaders
    renderingProgram = createShaderProgram(vertShaderFile,fragShderFile);

    // Generate perspective matrix
    perspective_matrix(window);

    // Create a vertex array object
    // Required by OpenGL to transfer data to the GPU
    glGenVertexArrays(numVAOs,vao);
    glBindVertexArray(vao[0]);

    // Create two vertex array buffers
    // The first will store point vertices
    // The second will store texture coordinates
    glGenBuffers(numVBOs,vbo);
    glBindBuffer(GL_ARRAY_BUFFER,vbo[0]);
    glBufferData(GL_ARRAY_BUFFER,maxPointsX3*sizeof(float),nullptr,GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER,vbo[1]);
    glBufferData(GL_ARRAY_BUFFER,maxPointsX3*sizeof(float),nullptr,GL_DYNAMIC_DRAW);

    // Set global texture mode variable
    textureMode = textureMode_;

    // Use user specified texture instead of images generated by camera
    if(textureMode == Image){
        textureRef = loadTexture(textureImage);
    }
}

void display(GLFWwindow* window, double currentTime, rs2::frameset &frames){
    //Update points and corresponding texture locations
    updateFrame(frames);
    
    glClear(GL_DEPTH_BUFFER_BIT);
    glClearColor(0.0,0.0,0.0,1.0);
    glClear(GL_COLOR_BUFFER_BIT);
    
    glUseProgram(renderingProgram);
    
    //get uniform variables for the MV and projection matrices
    mvLoc = glGetUniformLocation(renderingProgram, "mv_matrix");
    projLoc = glGetUniformLocation(renderingProgram, "proj_matrix");

    model_view_matrix();

    //printf("X offset: %f Y offset: %f Pitch: %f Yaw: %f\n", windowState.offset_x, windowState.offset_y, windowState.pitch, windowState.yaw);
//    glPointSize(width/640);
    glPointSize(1.0);

    //copy perspective and MV matrices
    glUniformMatrix4fv(mvLoc,1,GL_FALSE,value_ptr(mvMat));
    glUniformMatrix4fv(projLoc,1,GL_FALSE,value_ptr(pMat));

    glBindBuffer(GL_ARRAY_BUFFER,vbo[0]);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,0,0);
    glEnableVertexAttribArray(0);
    
    glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
    glUniform1i(glGetUniformLocation(renderingProgram,"samp"), 0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(1);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, textureRef);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDrawArrays(GL_POINTS, 0, maxPoints);

}








