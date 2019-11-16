//
// Global and state variables for OpenGL
//


#ifndef REALSENSE2_APP_STATE_H
#define REALSENSE2_APP_STATE_H

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <vector>
#include <librealsense2/rs.hpp>

enum TextureMode { Color = 1, Depth = 2, Image = 3};
extern int textureMode;
extern glm::mat4 pMat, vMat, mMat, mvMat; //perspective, view, mode, and model-view matrices

//Modified from https://github.com/IntelRealSense/librealsense/blob/master/examples/example.hpp
struct window_state {
    window_state(double yaw = -0, double pitch = -0) :  yaw(yaw), pitch(pitch), last_x(0.0), last_y(0.0),
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
extern long int max_, min_;
extern glm::vec4 point_1, point_2;

#endif //REALSENSE2_APP_STATE_H
