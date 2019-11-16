//
// Created by hayley on 11/16/19.
//

#ifndef REALSENSE2_PCL_UTILITIES_H
#define REALSENSE2_PCL_UTILITIES_H

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <librealsense2/rs.hpp>

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

pcl_ptr points_to_pcl(const rs2::points& points);
pcl_ptr create_mesh(const pcl_ptr& cloud);

#endif //REALSENSE2_PCL_UTILITIES_H
