//
// Created by hayley on 11/20/19.
//

#ifndef SHADOWS_R_REALSENSE_H
#define SHADOWS_R_REALSENSE_H

#include "renderable.h"
#include "pointlight.h"
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
    r_Realsense();
    ~r_Realsense() {};

    void setup();
    void set_description(std::string &description);
    void refresh(rs2::frameset &frames);

private:
    void updatePoints(pcl::PolygonMesh &triangles);
    void removeOldData();
    void addNewData(rs2::frameset &frames);

};

#endif //SHADOWS_R_REALSENSE_H
