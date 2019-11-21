//
// Created by hayley on 11/20/19.
//

#ifndef SHADOWS_APP_STATE_H
#define SHADOWS_APP_STATE_H


#include <GL/glew.h>
#include <glm/glm.hpp>
#include <vector>
#include <librealsense2/rs.hpp>

enum TextureMode { Color = 1, Image = 2};
extern int textureMode;
extern glm::mat4 pMat, vMat, mMat, mvMat; //perspective, view, mode, and model-view matrices

//Modified from https://github.com/IntelRealSense/librealsense/blob/master/examples/example.hpp
struct window_state {
    window_state(double yaw = 0, double pitch = 0) :  yaw(yaw), pitch(pitch), last_x(0.0), last_y(0.0),
                                                      ml(false), offset_x(0.f), offset_y(1.f) {}
    double yaw;
    double pitch;
    double last_x;
    double last_y;
    bool ml;
    float offset_x;
    float offset_y;
};

//Initialize state to hold window information
extern window_state windowState;
extern std::string textureImage;
extern long int max_z, min_z;
extern glm::vec4 point_1, point_2;

#endif //SHADOWS_APP_STATE_H
