//
// Created by hayley on 1/15/20.
//

#ifndef SHADOWS_APP_STATE_H
#define SHADOWS_APP_STATE_H

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <vector>
#include <librealsense2/rs.hpp>

enum TextureMode { Color = 1, Image = 2};
extern int textureMode;

#define REALSENSE  // DEFINE THIS IF USING THE REALSENSE - used to switch projection modes in camera, and handlers in main.


//Modified from https://github.com/IntelRealSense/librealsense/blob/master/examples/example.hpp
class window_state {
public:
    window_state() {
        yaw = 0.0;
        pitch = 0.0;
        last_x = 0.0;
        last_y = 0.0;
        ml = false;
        offset_x = 0.f;
        offset_y = 1.f;
        height = 0.0;
        width = 0.0;
        fov = glm::radians(60.0f);
        z_near = 0.01f;
        z_far = 10.0f;
        update_realsense = false;
    }
    float get_aspect_ratio(){
        return (float) width / (float) height;
    }

    float get_fov(){
        return fov;
    }

    float get_z_near() {
        return z_near;
    }

    float get_z_far(){
        return z_far;
    }

    double yaw;
    double pitch;
    double last_x;
    double last_y;
    bool ml;
    float offset_x;
    float offset_y;
    float height;
    float width;
    float fov;
    float z_near;
    float z_far;
    bool update_realsense;
};

//Initialize state to hold window information
extern window_state windowState;
extern const char * textureImage;


#endif //SHADOWS_APP_STATE_H
