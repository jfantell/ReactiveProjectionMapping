//
// Created by John Fantell on 11/25/19.
//

#ifndef SHADOWS_DRAW_H
#define SHADOWS_DRAW_H

#include <iostream>
#include "app_state.h"
#include "pointlight.h"
#include <glm/glm.hpp>
#include <pcl/point_types.h>
#include <librealsense2/rs.hpp>
#include <pcl/io/pcd_io.h>
#include "pcl_wrapper.h"
#include "renderable.h"
#include <glm/gtc/type_ptr.hpp>

class Draw {
public:
    Draw(GLuint shaderShadowProgramID, GLuint shaderPrimaryProgramID, Camera* camera, PointLight* light);
    ~Draw() {};
    void setup_shadow_buffers();
    void add_models(std::vector<Renderable *> renderable_models);
    void draw();

private:
    //vector of pointers to all models
    std::vector<Renderable *> _renderable_models;
    int _num_models;

    //attach programs
    GLuint _shaderId;
    GLuint _shadowShaderId;

    //for shadows
    glm::mat4 _lightVmatrix;
    glm::mat4 _lightPmatrix;
    GLuint _shadowBuffer;
    GLuint _shadowTex;
    glm::mat4 _b;

    //create uniform linker variables
    GLuint _shaderModelId = 0;
    GLuint _shaderViewId = 0;
    GLuint _shaderModelViewId = 0;
    GLuint _shaderPerpectiveId = 0;
    GLuint _shadowMVPId = 0;
    GLuint _invTrMVId = 0;
    GLuint _shadowShaderID = -1;
    GLuint _shaderUseTextureId = 0;
    GLuint _vaoId = 0;
    GLuint _texId = 0;
    Camera* _camera;
    PointLight* _light;
    GLuint useTexture = 0;

    void pass_one(Renderable * model);
    void pass_two(Renderable * model);

};

#endif //SHADOWS_DRAW_H
