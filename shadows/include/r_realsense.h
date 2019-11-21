//
// Created by hayley on 11/20/19.
//

#ifndef SHADOWS_R_REALSENSE_H
#define SHADOWS_R_REALSENSE_H

#include "renderable.h"
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

private:
    GLuint _shaderMVPId = 0;
    GLuint _shaderModelId = 0;
    GLuint _shaderUseTextureId = 0;
    GLuint _vaoId = 0;
    GLuint _texId = 0;
    std::string _objpath = "";
    std::string _texpath = "";
    Mesh _mesh;
    VertexBuffer _vb;
    IndexBuffer _ib;

    void updatePoints(pcl::PolygonMesh &triangles);
    void clearDataVectors();
    void updateFrame(rs2::frameset &frames);
    void draw_buffers();
};

#endif //SHADOWS_R_REALSENSE_H
