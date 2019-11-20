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
#define numVBOs 5
#define MAX_SIZE 100000

GLuint renderingProgram; // Vertex and Fragment Shader Program
GLuint vao[numVAOs]; // Array to store Vertex Attribute Objects References
GLuint vbo[numVBOs]; //Array to store Vertex Attribute Buffer References
GLuint mvLoc, projLoc, normLoc, useTextureLoc; // Connect C++ program to Shaders
GLsizei num_vertices; //The number of points to be drawn
GLuint textureRef = 0; // Reference to texture address in memory
std::vector<float> vertexCoordinatesVector;
std::vector<float> RGBcolorVector;
std::vector<float> textureCoordinatesVector;
std::vector<float> vertexNormVector;
std::vector<unsigned int> vertexIndicesVector;

rs2::pointcloud pc; // Point cloud object
rs2::points points; // RealSense points object

glm::mat4 mvMat, vMat, mMat, pMat; //Define global variables
int useTexture = 0;
window_state windowState;
vec4 point_1, point_2;

void updatePoints(pcl::PolygonMesh &triangles){
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_from_mesh (new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromPCLPointCloud2 (triangles.cloud, *cloud_from_mesh);

//    cout << "Cloud Data " << cloud_from_mesh->size() << endl;
//    cout << "Polygons Data " << triangles.polygons.size() << endl;

    //Load vertex coordinates into vertex buffer
    //Load corresponding RGB values into color buffer
    //Compute min and max x,y values as described by
    //https://stackoverflow.com/questions/35141677/opengl-c-generating-uv-coordinates
    float min_X = 1000000000;
    float max_X = -1000000000;
    float min_Y = 1000000000;
    float max_Y = -1000000000;

    cout << "CLOUD SIZE " << cloud_from_mesh->size() << endl;
    for (int i=0; i < cloud_from_mesh->size(); i++)
    {
        float x = cloud_from_mesh->points[i].x;
        float y = cloud_from_mesh->points[i].y;
        float z = cloud_from_mesh->points[i].z;
        float x_normal = cloud_from_mesh->points[i].normal_x;
        float y_normal = cloud_from_mesh->points[i].normal_y;
        float z_normal = cloud_from_mesh->points[i].normal_z;
        if( i == 0){
            point_1 = vec4(x,y,z,1);
        }
        if( i == 200){
            point_2 = vec4(x,y,z,1);
        }

        vertexCoordinatesVector.push_back(x);
        vertexCoordinatesVector.push_back(y);
        vertexCoordinatesVector.push_back(z);

        //Assign normals to each vertex
        vertexNormVector.push_back(x_normal);
        vertexNormVector.push_back(y_normal);
        vertexNormVector.push_back(z_normal);

        //Assign RGB colors to each vertex
        RGBcolorVector.push_back(1);
        RGBcolorVector.push_back(1);
        RGBcolorVector.push_back(1);

        min_X = std::min(min_X,x);
        max_X = std::max(max_X,x);
        min_Y = std::min(min_Y,y);
        max_Y = std::max(max_Y,y);
    }

    //Generate scaling values for texture coordinate generation
    float k_X = 1/(max_X-min_X);
    float k_Y = 1/(max_Y-min_Y);

    //Generate texture coordinates
    for (int i=0; i < cloud_from_mesh->size(); i++)
    {
        float x = cloud_from_mesh->points[i].x;
        float y = cloud_from_mesh->points[i].y;
        float u = (x-min_X) * k_X;
        float v = (y-min_Y) * k_Y;
        textureCoordinatesVector.push_back(u);
        textureCoordinatesVector.push_back(v);
    }

    // Load polygons indices into element array buffer
    for(int i=0; i<triangles.polygons.size(); i++) {
        std::vector<unsigned int> indices = triangles.polygons[i].vertices;

        vertexIndicesVector.push_back(indices[0]);
        vertexIndicesVector.push_back(indices[1]);
        vertexIndicesVector.push_back(indices[2]);
//        cout << "Indices " << indices[0] << " " << indices[1] << " " << indices[2] << endl;
    }

    num_vertices = (GLsizei)triangles.polygons.size() * 3;
    cout << "Num Vertices " << num_vertices << endl;

    glBindBuffer(GL_ARRAY_BUFFER,vbo[0]);
    glBufferSubData(GL_ARRAY_BUFFER, 0, (long int)vertexCoordinatesVector.size() * sizeof(float), vertexCoordinatesVector.data());
    glBindBuffer(GL_ARRAY_BUFFER,vbo[1]);
    glBufferSubData(GL_ARRAY_BUFFER, 0, (long int)RGBcolorVector.size() * sizeof(float), RGBcolorVector.data());
    glBindBuffer(GL_ARRAY_BUFFER,vbo[2]);
    glBufferSubData(GL_ARRAY_BUFFER, 0, (long int)textureCoordinatesVector.size() * sizeof(float), textureCoordinatesVector.data());
    glBindBuffer(GL_ARRAY_BUFFER,vbo[3]);
    glBufferSubData(GL_ARRAY_BUFFER, 0, (long int)vertexNormVector.size() * sizeof(float), vertexNormVector.data());
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,vbo[4]);
    glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, (long int)vertexIndicesVector.size() * sizeof(unsigned int), vertexIndicesVector.data());

}

void clearCPUbuffers(){
    // Vectors store memory on the heap
    // this memory needs to be erased before
    // a new frame can be drawn
    vertexCoordinatesVector.clear();
    RGBcolorVector.clear();
    textureCoordinatesVector.clear();
    vertexNormVector.clear();
    vertexIndicesVector.clear();
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
    glBufferData(GL_ARRAY_BUFFER, MAX_SIZE * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER,vbo[1]);
    glBufferData(GL_ARRAY_BUFFER, MAX_SIZE * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER,vbo[2]);
    glBufferData(GL_ARRAY_BUFFER, MAX_SIZE * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER,vbo[3]);
    glBufferData(GL_ARRAY_BUFFER, MAX_SIZE * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,vbo[4]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, MAX_SIZE * sizeof(unsigned int), nullptr, GL_DYNAMIC_DRAW);

    // Use user specified texture instead of images generated by camera
    if(textureMode == Image){
        useTexture = 1;
        textureRef = loadTexture(textureImage);
    }
}

void setup_buffers(){
    glClear(GL_DEPTH_BUFFER_BIT);
    glClearColor(0.0,0.0,0.0,1.0);
    glClear(GL_COLOR_BUFFER_BIT);

    glUseProgram(renderingProgram);

    //get uniform variable locations in GPU
    mvLoc = glGetUniformLocation(renderingProgram, "mv_matrix");
    projLoc = glGetUniformLocation(renderingProgram, "proj_matrix");
    normLoc = glGetUniformLocation(renderingProgram, "norm_matrix");
    useTextureLoc = glGetUniformLocation(renderingProgram, "use_texture");

    model_view_matrix();

    glPointSize(1.0);

    //copy uniforms from C++ to GLSL
    glUniformMatrix4fv(mvLoc,1,GL_FALSE,value_ptr(mvMat));
    glUniformMatrix4fv(projLoc,1,GL_FALSE,value_ptr(pMat));
    glUniform1i(useTextureLoc,useTexture);

    //Vertex buffer
    glBindBuffer(GL_ARRAY_BUFFER,vbo[0]);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,0,0);
    glEnableVertexAttribArray(0);

    //Color buffer
    glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(1);

    //Texture Buffer
    glBindBuffer(GL_ARRAY_BUFFER, vbo[2]);
    glUniform1i(glGetUniformLocation(renderingProgram,"samp"), 0);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(2);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, textureRef);

    //Norm buffer
    glBindBuffer(GL_ARRAY_BUFFER, vbo[3]);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(3);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo[4]);
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









