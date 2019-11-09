//
// Created by hayley on 11/20/19.
//

#ifndef SHADOWS_PCL_WRAPPER_H
#define SHADOWS_PCL_WRAPPER_H

#include <pcl/point_types.h>
#include <librealsense2/rs.hpp>
#include <pcl/io/pcd_io.h>

class PCL_wrapper{
public:
    PCL_wrapper();
    ~PCL_wrapper();
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points& points, const rs2::video_frame& color);
    pcl::PolygonMesh create_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_xyz);

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr octree_voxel_downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr passthrough_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr statistical_outlier_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    pcl::PointCloud<pcl::PointNormal>::Ptr estimate_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    pcl::PolygonMesh greedy_projection_mesh_reconstruction(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_with_normals);
    void save_cloud(const char * filepath, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

};

#endif //SHADOWS_PCL_WRAPPER_H
