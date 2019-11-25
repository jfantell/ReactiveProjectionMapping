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

GLuint vao[numVAOs]; // Array to store Vertex Attribute Objects References
GLuint vbo[numVBOs]; //Array to store Vertex Attribute Buffer References

GLsizei num_vertices; //The number of points to be drawn
GLuint textureRef = 0; // Reference to texture address in memory
std::vector<float> vertexCoordinatesVector;
std::vector<float> RGBcolorVector;
std::vector<float> textureCoordinatesVector;
std::vector<float> vertexNormVector;
std::vector<unsigned int> vertexIndicesVector;

//For shadows
GLuint shadowTex, shadowBuffer;
glm::mat4 lightVmatrix;
glm::mat4 lightPmatrix;

glm::mat4 b;

rs2::pointcloud pc; // Point cloud object
rs2::points points; // RealSense points object

GLuint useTexture = 0;
window_state windowState;
glm::vec4 point_1, point_2;

using namespace std;

r_Realsense::r_Realsense(GLuint shaderProgramId, Camera *camera) {
    _shaderId = shaderProgramId;
    _shadowShaderID = -1;
    _light = NULL;
    _camera = camera;
    _transform = Transform();
    _mesh = Mesh();
    _vb = VertexBuffer();
    _ib = IndexBuffer();
}

void r_Realsense::setupShadowBuffers(){
    glGenFramebuffers(1, &shadowBuffer);

    glGenTextures(1, &shadowTex);
    glBindTexture(GL_TEXTURE_2D, shadowTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32,
                 windowState.width, windowState.height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);

    // may reduce shadow border artifacts
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

}

void r_Realsense::setup() {
    /////// BUFFER SETUP
    // Create a vertex array object
    // Required by OpenGL to transfer data to the GPU
    glGenVertexArrays(numVAOs,&_vaoId);
    glBindVertexArray(_vaoId);

    // Create two vertex array buffers
    // The first will store point vertices
    // The second will store texture coordinates
    glGenBuffers(numVBOs,vbo);
    glBindBuffer(GL_ARRAY_BUFFER,vbo[0]); //vert
    glBufferData(GL_ARRAY_BUFFER, MAX_SIZE * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER,vbo[1]); //normals
    glBufferData(GL_ARRAY_BUFFER, MAX_SIZE * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER,vbo[2]); //text
    glBufferData(GL_ARRAY_BUFFER, MAX_SIZE * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER,vbo[3]); //colors
    glBufferData(GL_ARRAY_BUFFER, MAX_SIZE * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,vbo[4]); //indicies
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, MAX_SIZE * sizeof(unsigned int), nullptr, GL_DYNAMIC_DRAW);

    ////// TEXTURE MAPPING SETUP
    if(textureMode == Image){
        useTexture = 1;
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

    ////// SHADOW SETUP
    setupShadowBuffers();
    b = glm::mat4(
            0.5f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.5f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.5f, 0.0f,
            0.5f, 0.5f, 0.5f, 1.0f);

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
        if( i == 0){
            point_1 = glm::vec4(x,y,z,1);
        }
        if( i == 200){
            point_2 = glm::vec4(x,y,z,1);
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
//    cout << "Num Vertices " << num_vertices << endl;

    glBindBuffer(GL_ARRAY_BUFFER,vbo[0]); //vert
    glBufferSubData(GL_ARRAY_BUFFER, 0, (long int)vertexCoordinatesVector.size() * sizeof(float), vertexCoordinatesVector.data());
    glBindBuffer(GL_ARRAY_BUFFER,vbo[1]); //norm
    glBufferSubData(GL_ARRAY_BUFFER, 0, (long int)vertexNormVector.size() * sizeof(float), vertexNormVector.data());
    glBindBuffer(GL_ARRAY_BUFFER,vbo[2]); //text
    glBufferSubData(GL_ARRAY_BUFFER, 0, (long int)textureCoordinatesVector.size() * sizeof(float), textureCoordinatesVector.data());
    glBindBuffer(GL_ARRAY_BUFFER,vbo[3]); //colors
    glBufferSubData(GL_ARRAY_BUFFER, 0, (long int)RGBcolorVector.size() * sizeof(float), RGBcolorVector.data());
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,vbo[4]); //indices
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

void r_Realsense::add_shadow_shader(GLuint shadow_shaderProgramId){
    _shadowShaderID = shadow_shaderProgramId;
}

void r_Realsense::add_light(PointLight* light){
    _light = light;
}

void r_Realsense::pass_one() {
    glUseProgram(_shadowShaderID);

    glm::mat4 shadowMVP1 = lightPmatrix * lightVmatrix * _transform.getModelMatrix();
    _shadowMVPId = glGetUniformLocation(_shadowShaderID, "Shadow_MVP");
    glUniformMatrix4fv(_shadowMVPId, 1, GL_FALSE, glm::value_ptr(shadowMVP1));

    //Vertex Buffer
    glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(0);

    glClear(GL_DEPTH_BUFFER_BIT);
//    glEnable(GL_CULL_FACE);
//    glFrontFace(GL_CCW);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    //Element Array Buffer
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo[4]);
    glDrawElements(GL_TRIANGLES,num_vertices,GL_UNSIGNED_INT, 0);
}

void r_Realsense::pass_two() {
    glUseProgram(_shaderId);

//    //Bind VAO
//    glBindVertexArray(_vaoId);

    _shaderModelId = glGetUniformLocation(_shaderId, "Model");
    _shaderViewId = glGetUniformLocation(_shaderId, "View");
    _shaderModelViewId = glGetUniformLocation(_shaderId, "ModelView");
    _shaderPerpectiveId = glGetUniformLocation(_shaderId, "Perpective");
    _shaderUseTextureId = glGetUniformLocation(_shaderId, "Use_Text");
    _shadowMVPId = glGetUniformLocation(_shaderId, "Shadow_MVP");
    _invTrMVId = glGetUniformLocation(_shaderId, "InvTr_MV");

    glm::mat4 shadowMVP2 = b * lightPmatrix * lightVmatrix * _transform.getModelMatrix();
    glm::mat4 mvMat = _camera->getView() * _transform.getModelMatrix();
    glm::mat4 invTrMat = glm::transpose(glm::inverse(mvMat));

    glUniformMatrix4fv(_shaderModelId, 1, GL_FALSE, glm::value_ptr( _transform.getModelMatrix()));
    glUniformMatrix4fv(_shaderViewId, 1, GL_FALSE, glm::value_ptr( _camera->getView()));
    glUniformMatrix4fv(_shaderModelViewId, 1, GL_FALSE, glm::value_ptr( mvMat));
    glUniformMatrix4fv(_shaderPerpectiveId, 1, GL_FALSE, glm::value_ptr(_camera->getProjection()));
    glUniformMatrix4fv(_shadowMVPId, 1, GL_FALSE, glm::value_ptr(shadowMVP2));
    glUniformMatrix4fv(_invTrMVId, 1, GL_FALSE, glm::value_ptr(invTrMat));
    glUniform1i(_shaderUseTextureId,useTexture);

//    glBindTexture(GL_TEXTURE_2D, _texId);
////    glBindVertexArray(_vaoId);

    //Vertex buffer
    glBindBuffer(GL_ARRAY_BUFFER,vbo[0]);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,0,0);
    glEnableVertexAttribArray(0);

    //Norm buffer
    glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(1);

    //Texture Buffer
    glBindBuffer(GL_ARRAY_BUFFER, vbo[2]);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(2);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, _texId);
    glUniform1i(glGetUniformLocation(_shaderId, "textureimage"), 1);

    //Color buffer
    glBindBuffer(GL_ARRAY_BUFFER, vbo[3]);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(3);

    glClear(GL_DEPTH_BUFFER_BIT);
//    glEnable(GL_CULL_FACE);
//    glFrontFace(GL_CCW);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    // Element array buffer
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo[4]);
    glDrawElements(GL_TRIANGLES,num_vertices,GL_UNSIGNED_INT, 0);
}

void r_Realsense::draw() {}

void r_Realsense::draw(rs2::frameset &frames) {
    // Remove old camera data and add new camera data to vectors
    removeOldData();
    addNewData(frames);

    // Set up light view matrix and perpective matrix
    glm::vec3 up(0.0f, -1.0f, 0.0f);
    glm::vec3 current_loc_light = _light->getWorldLocation();
    glm::vec3 current_loc_model = _transform.getWorldPosition();
    lightVmatrix = glm::lookAt(current_loc_light, current_loc_model, up);
    lightPmatrix = glm::perspective(windowState.get_fov(), windowState.get_aspect_ratio(), windowState.get_z_near(), windowState.get_z_far());

    glBindFramebuffer(GL_FRAMEBUFFER, shadowBuffer);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, shadowTex, 0);

    glDrawBuffer(GL_NONE);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_POLYGON_OFFSET_FILL);	// for reducing
    glPolygonOffset(2.0f, 4.0f);		//  shadow artifacts

    pass_one();

    glDisable(GL_POLYGON_OFFSET_FILL);	// artifact reduction, continued

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, shadowTex);
    glUniform1i(glGetUniformLocation(_shadowShaderID, "shadowTex"), 0);

    glDrawBuffer(GL_FRONT);

    pass_two();
}