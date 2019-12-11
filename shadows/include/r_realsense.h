#ifndef SHADOWS_R_REALSENSE_H
#define SHADOWS_R_REALSENSE_H

#include "renderable.h"
#include "pointlight.h"
#include "GL/glew.h"
#include <iostream>
#include "camera.h"
#include <mesh.h>
#include <librealsense2/rs.hpp>
#include "vertexbuffer.h"
#include "indexbuffer.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

class r_Realsense : public Renderable {
public:
    r_Realsense(GLuint shaderProgramId, Camera * camera);
    ~r_Realsense() {}; // Overrides ~Renderable()

    void setup(); // Overrides Renderable::setup();
    void draw();
    void draw(rs2::frameset &frames);
    void add_shadow_shader(GLuint shadow_shaderProgramId);
    void add_light(PointLight* light);

private:
    GLuint _shaderModelId = 0;
    GLuint _shaderViewId = 0;
    GLuint _shaderModelViewId = 0;
    GLuint _shaderPerpectiveId = 0;
    GLuint _shadowMVPId = 0;
    GLuint _invTrMVId = 0;
    GLuint _shadowShaderID = -1;
    PointLight* _light = NULL;
    GLuint _shaderUseTextureId = 0;
    GLuint _vaoId = 0;
    GLuint _texId = 0;
    std::string _objpath = "";
    std::string _texpath = "";
    Mesh _mesh;
    VertexBuffer _vb;
    IndexBuffer _ib;

    void updatePoints(pcl::PolygonMesh &triangles);
    void removeOldData();
    void addNewData(rs2::frameset &frames);
    void draw_buffers();
    void pass_one();
    void pass_two();
    void setupShadowBuffers();
};

#endif //SHADOWS_R_REALSENSE_H
