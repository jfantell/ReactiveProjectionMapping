//
// Created by hayley on 11/20/19.
//

#include "r_realsense.h"
#include "r_objmodel.h"
#include "stb_image.h"
#include "vertexbuffer.h"
#include "indexbuffer.h"
#include <iostream>
#include "app_state.h"
#include <glm/glm.hpp>
#include <pcl/point_types.h>
#include <librealsense2/rs.hpp>
#include <pcl/io/pcd_io.h>
#include "pcl_wrapper.h"
#include <glm/gtc/type_ptr.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <GL/glew.h>

#define numVAOs 1
#define numVBOs 5
#define MAX_SIZE 100000


GLsizei num_vertices; //The number of points to be drawn
GLuint textureRef = 0; // Reference to texture address in memory
std::vector<float> vertexCoordinatesVector;
std::vector<float> RGBcolorVector;
std::vector<float> textureCoordinatesVector;
std::vector<float> vertexNormVector;
std::vector<unsigned int> vertexIndicesVector;

rs2::pointcloud pc; // Point cloud object
rs2::points points; // RealSense points object

window_state windowState;

using namespace std;

r_Realsense::r_Realsense() {
    _transform = Transform();
    _vao[numVAOs];
    _vbo[numVBOs];
}

void r_Realsense::setup() {
    /////// BUFFER SETUP
    // Create a vertex array object
    // Required by OpenGL to transfer data to the GPU
    glGenVertexArrays(numVAOs,_vao);
    glBindVertexArray(*_vao);

    // Create two vertex array buffers
    // The first will store point vertices
    // The second will store texture coordinates
    glGenBuffers(numVBOs,_vbo);
    glBindBuffer(GL_ARRAY_BUFFER,_vbo[0]); //vert
    glBufferData(GL_ARRAY_BUFFER, MAX_SIZE * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER,_vbo[1]); //normals
    glBufferData(GL_ARRAY_BUFFER, MAX_SIZE * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER,_vbo[2]); //text
    glBufferData(GL_ARRAY_BUFFER, MAX_SIZE * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER,_vbo[3]); //colors
    glBufferData(GL_ARRAY_BUFFER, MAX_SIZE * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,_vbo[4]); //indicies
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, MAX_SIZE * sizeof(unsigned int), nullptr, GL_DYNAMIC_DRAW);

    ////// TEXTURE MAPPING SETUP
    if(windowState.useTexture == 1){
        int width, height, nrChannels;
        unsigned char *data = stbi_load(textureImage, &width, &height, &nrChannels, 0);
        if(data) {
            glGenTextures(1, &_texId);
            glBindTexture(GL_TEXTURE_2D, _texId);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);

            // set the texture wrapping/filtering options (on the currently bound texture object)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            glGenerateMipmap(GL_TEXTURE_2D);
            if (glewIsSupported("GL_EXT_texture_filter_anisotropic")) {
                GLfloat anisoset = 0.0f;
                glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &anisoset);
                glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, anisoset);
            }
        }
        else
        {
            std::cerr << "Failed to load texture Realsense" << textureImage << std::endl;
            exit(-1);
        }
        stbi_image_free(data);
    }
}

void r_Realsense::updatePoints(pcl::PolygonMesh &triangles){
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_from_mesh (new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromPCLPointCloud2 (triangles.cloud, *cloud_from_mesh);

    cout << "Vertex Data " << cloud_from_mesh->size() << endl;
    cout << "Polygons Data " << triangles.polygons.size() << endl;

    //Load vertex coordinates into vertex buffer
    //Load corresponding RGB values into color buffer
    //Compute min and max x,y values as described by
    //https://stackoverflow.com/questions/35141677/opengl-c-generating-uv-coordinates
    float min_X = 1000000000;
    float max_X = -1000000000;
    float min_Y = 1000000000;
    float max_Y = -1000000000;

//    cout << "CLOUD SIZE " << cloud_from_mesh->size() << endl;
    for (int i=0; i < cloud_from_mesh->size(); i++)
    {
        float x = cloud_from_mesh->points[i].x;
        float y = cloud_from_mesh->points[i].y;
        float z = cloud_from_mesh->points[i].z;
        float x_normal = cloud_from_mesh->points[i].normal_x;
        float y_normal = cloud_from_mesh->points[i].normal_y;
        float z_normal = cloud_from_mesh->points[i].normal_z;

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
//    cout << "Num Vertices " << num_vertices << endl;

    glBindBuffer(GL_ARRAY_BUFFER,_vbo[0]); //vert
    glBufferSubData(GL_ARRAY_BUFFER, 0, (long int)vertexCoordinatesVector.size() * sizeof(float), vertexCoordinatesVector.data());
    glBindBuffer(GL_ARRAY_BUFFER,_vbo[1]); //norm
    glBufferSubData(GL_ARRAY_BUFFER, 0, (long int)vertexNormVector.size() * sizeof(float), vertexNormVector.data());
    glBindBuffer(GL_ARRAY_BUFFER,_vbo[2]); //text
    glBufferSubData(GL_ARRAY_BUFFER, 0, (long int)textureCoordinatesVector.size() * sizeof(float), textureCoordinatesVector.data());
    glBindBuffer(GL_ARRAY_BUFFER,_vbo[3]); //colors
    glBufferSubData(GL_ARRAY_BUFFER, 0, (long int)RGBcolorVector.size() * sizeof(float), RGBcolorVector.data());
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,_vbo[4]); //indices
    glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, (long int)vertexIndicesVector.size() * sizeof(unsigned int), vertexIndicesVector.data());

}

void r_Realsense::removeOldData(){
    // Vectors store memory on the heap
    // this memory needs to be erased before
    // a new frame can be drawn
    vertexCoordinatesVector.clear();
    RGBcolorVector.clear();
    textureCoordinatesVector.clear();
    vertexNormVector.clear();
    vertexIndicesVector.clear();
}
void r_Realsense::addNewData(rs2::frameset &frames){
    // Wait for the next set of frames from the RealSense Camera
    auto color = frames.get_color_frame();

    // Tell pointcloud object to map to this color frame
    pc.map_to(color);

    auto depth = frames.get_depth_frame();

    // Generate the pointcloud and texture mappings
    points = pc.calculate(depth);

    // Convert RealSense point cloud to PCL point cloud
    PCL_wrapper pcl_wrapper = PCL_wrapper();
    auto pcl_points = pcl_wrapper.points_to_pcl(points,color);

    //Create mesh
    pcl::PolygonMesh triangles = pcl_wrapper.create_mesh(pcl_points);
    updatePoints(triangles);

}

void r_Realsense::refresh(rs2::frameset &frames) {
    // Remove old camera data and add new camera data to vectors
    removeOldData();
    addNewData(frames);
}

void r_Realsense::set_description(std::string &description) {
    _description = description;
}