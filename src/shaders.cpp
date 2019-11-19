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
#define numVBOs 3
#define maxPointsX3 2800000

GLuint renderingProgram; // Vertex and Fragment Shader Program
GLuint vao[numVAOs]; // Array to store Vertex Attribute Objects References
GLuint vbo[numVBOs]; //Array to store Vertex Attribute Buffer References
GLuint mvLoc, projLoc; // Connect C++ program to Shaders
GLsizei num_vertices; //The number of points to be drawn
std::vector<float> vertexCoordinatesVector;
std::vector<float> RGBcolorVector;
std::vector<unsigned int> vertexIndicesVector;

rs2::pointcloud pc; // Point cloud object
rs2::points points; // RealSense points object

glm::mat4 mvMat, vMat, mMat, pMat; //Define global variables
window_state windowState;
vec4 point_1, point_2;

void updatePoints(pcl::PolygonMesh &triangles){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_from_mesh (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2 (triangles.cloud, *cloud_from_mesh);

//    cout << "Cloud Data " << cloud_from_mesh->size() << endl;
//    cout << "Polygons Data " << triangles.polygons.size() << endl;

    //Load vertex coordinates into vertex buffer
    //Load corresponding RGB values into color buffer
    for (int i=0; i < cloud_from_mesh->size(); i++)
    {
        float x = cloud_from_mesh->points[i].x;
        float y = cloud_from_mesh->points[i].y;
        float z = cloud_from_mesh->points[i].z;
        if( i == 0){
            point_1 = vec4(x,y,z,1);
        }
        if( i == 200){
            point_2 = vec4(x,y,z,1);
        }

//        uint8 r = cloud_from_mesh->points[i].r;
//        Eigen::Vector3i color = cloud_from_mesh->points[i].getRGBVector3i();
//        float r = color[0];
//        float g = color[1];
//        float b = color[2];
//        cout << "Colors " << r/255 << " " << g/255 << " " << b/255 << endl;
//        cout << "Coordinates " << x << " " << y << " " << z << endl;

        vertexCoordinatesVector.push_back(x);
        vertexCoordinatesVector.push_back(y);
        vertexCoordinatesVector.push_back(z);
        RGBcolorVector.push_back(1);
        RGBcolorVector.push_back(1);
        RGBcolorVector.push_back(1);
    }
//    getchar();


    // Load polygons indices into element array buffer
    for(int i=0; i<triangles.polygons.size(); i++) {
        std::vector<unsigned int> indices = triangles.polygons[i].vertices;

        vertexIndicesVector.push_back(indices[0]);
        vertexIndicesVector.push_back(indices[1]);
        vertexIndicesVector.push_back(indices[2]);
//        cout << "Indices " << indices[0] << " " << indices[1] << " " << indices[2] << endl;
    }


    num_vertices = (GLsizei)triangles.polygons.size();
    cout << "Num Vertices " << num_vertices << endl;

    glBindBuffer(GL_ARRAY_BUFFER,vbo[0]);
    glBufferSubData(GL_ARRAY_BUFFER, 0, (long int)vertexCoordinatesVector.size() * sizeof(float), vertexCoordinatesVector.data());
    glBindBuffer(GL_ARRAY_BUFFER,vbo[1]);
    glBufferSubData(GL_ARRAY_BUFFER, 0, (long int)RGBcolorVector.size() * sizeof(float), RGBcolorVector.data());
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,vbo[2]);
    glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, (long int)vertexIndicesVector.size() * sizeof(unsigned int), vertexIndicesVector.data());

}

void clearCPUbuffers(){
    // Vectors store memory on the heap
    // this memory needs to be erased before
    // a new frame can be drawn
    if(!vertexCoordinatesVector.empty()){
        vertexCoordinatesVector.clear();
    }
    if(!RGBcolorVector.empty()){
        RGBcolorVector.clear();
    }
    if(!vertexIndicesVector.empty()){
        vertexIndicesVector.clear();
    }
}

void updateFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
    pcl::PolygonMesh triangles = create_mesh(cloud);
    updatePoints(triangles);

}

void updateFrame(rs2::frameset &frames){
    // Wait for the next set of frames from the RealSense Camera
    auto color = frames.get_color_frame();

    // Tell pointcloud object to map to this color frame
    pc.map_to(color);

    auto depth = frames.get_depth_frame();

    // Generate the pointcloud and texture mappings
    points = pc.calculate(depth);

    // Convert RealSense point cloud to PCL point cloud
    auto pcl_points = points_to_pcl(points,color);

    //Create mesh
    pcl::PolygonMesh triangles = create_mesh(pcl_points);
    updatePoints(triangles);

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
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,vbo[2]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,maxPointsX3*sizeof(unsigned int),nullptr,GL_DYNAMIC_DRAW);
}
void setup_buffers(){
    glClear(GL_DEPTH_BUFFER_BIT);
    glClearColor(0.0,0.0,0.0,1.0);
    glClear(GL_COLOR_BUFFER_BIT);

    glUseProgram(renderingProgram);

    //get uniform variables for the MV and projection matrices
    mvLoc = glGetUniformLocation(renderingProgram, "mv_matrix");
    projLoc = glGetUniformLocation(renderingProgram, "proj_matrix");

    model_view_matrix();

    glPointSize(1.0);

    //copy perspective and MV matrices
    glUniformMatrix4fv(mvLoc,1,GL_FALSE,value_ptr(mvMat));
    glUniformMatrix4fv(projLoc,1,GL_FALSE,value_ptr(pMat));

    glBindBuffer(GL_ARRAY_BUFFER,vbo[0]);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,0,0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(1);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo[2]);
    glDrawElements(GL_TRIANGLES,num_vertices,GL_UNSIGNED_INT, 0);
}

void display(GLFWwindow* window, double currentTime, rs2::frameset &frames){
    clearCPUbuffers();
    //Update points and corresponding texture locations
    updateFrame(frames);
    setup_buffers();
}

void display(GLFWwindow* window, double currentTime, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    clearCPUbuffers();
    //Update points and corresponding texture locations
    updateFrame(cloud);
    setup_buffers();
}









