//
// Created by hayley on 11/16/19.
//

#include <pcl/point_types.h>
#include <librealsense2/rs.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/vtk_io.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using namespace std;

////Source: https://github.com/IntelRealSense/librealsense/blob/master/wrappers/pcl/pcl/rs-pcl.cpp
////Source: https://github.com/eMrazSVK/JetsonSLAM/blob/master/pcl_testing.cpp
pcl_ptr points_to_pcl(const rs2::points& points, const rs2::video_frame& color)
{
    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto vertex_coordinates = points.get_vertices();
    auto texture_coordinates = points.get_texture_coordinates();
    for (int i=0; i < points.size(); i++) {
        //XYZ values
        cloud->points[i].x = vertex_coordinates[i].x;
        cloud->points[i].y = vertex_coordinates[i].y;
        cloud->points[i].z = vertex_coordinates[i].z;
    }
    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr uniform_sampler(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::UniformSampling< pcl::PointXYZ > filter;
    filter.setInputCloud(cloud);
    filter.setRadiusSearch(0.001);
    filter.filter(*cloud_filtered);
    cout << "Uniform Filtered Cloud" << endl;
    return cloud_filtered;
}

//Source: http://sideproject.acraig.za.net/2017/09/10/downsampling-point-clouds-with-pcl/
pcl::PointCloud<pcl::PointXYZ>::Ptr octree_voxel_downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
    float leaf = 0.05f;
    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> octree(leaf);
    octree.setInputCloud(cloud);
    octree.defineBoundingBox();
    octree.addPointsFromInputCloud();
    pcl::octree::OctreePointCloud<pcl::PointXYZ>::AlignedPointTVector centroids;
    octree.getVoxelCentroids(centroids);

    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
    output->points.assign(centroids.begin(), centroids.end());
    output->width = uint32_t(centroids.size());
    output->height = 1;
    output->is_dense = true;
    return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr passthrough_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.5);
    pass.filter (*cloud_filtered);
    cout << "PassThrough Filtered Cloud" << endl;
    return cloud_filtered;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr statistical_outlier_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);
    cout << "SOR Filtered Cloud" << endl;
    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud (cloud);
    vox.setLeafSize (0.03f, 0.03f, 0.03f);
    vox.filter (*cloud_filtered);
    cout << "VoxelGrid Downsampled Cloud" << endl;
    return cloud_filtered;
}

//Source: http://www.pointclouds.org/assets/icra2012/surface.pdf
pcl::PointCloud<pcl::PointXYZ>::Ptr movingleastsquares_upsample(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
    //Upsample using MovingLeastSquares
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    mls.setInputCloud (cloud);
    mls.setSearchRadius (1);
    mls.setPolynomialOrder (2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed (new pcl::PointCloud<pcl::PointXYZ> ());
    mls.process (*cloud_smoothed);
    return cloud_smoothed;
}

pcl::PointCloud<pcl::PointNormal>::Ptr estimate_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    n.setNumberOfThreads(16);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (50);
    n.compute (*normals);

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    cout << "Created and Concatenated Normals with Vertex Coordinates " << endl;
    return cloud_with_normals;
}

pcl::PolygonMesh greedy_projection_mesh_reconstruction(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_with_normals){
    //    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree_mesh (new pcl::search::KdTree<pcl::PointNormal>);
    tree_mesh->setInputCloud(cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (2);
    //    gp3.setSearchRadius (5);

    // Set typical values for the parameters
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (100);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree_mesh);
    gp3.reconstruct (triangles);

    // Additional vertex information
    cout << "Mesh Constructed" << endl;
    return triangles;
}

void save_cloud(const char * filepath, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
    //Save downsampled pointcloud
    pcl::PCLPointCloud2 cloud_blob_samp;
    pcl::toPCLPointCloud2 (*cloud, cloud_blob_samp);
    pcl::io::saveVTKFile (filepath, cloud_blob_samp);
    cout << "Cloud Saved" << endl;
}

//Source: http://www.pointclouds.org/documentation/tutorials/greedy_projection.php
pcl::PolygonMesh create_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_xyz) {
    cout << "Constructing Mesh" << endl;

    //Uniform Sampling
    pcl::PointCloud<pcl::PointXYZ>::Ptr uniform_sample = octree_voxel_downsample(cloud_xyz);

    //Pass through filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr pass_through = passthrough_filter(uniform_sample);

//    //Statistical Outlier
    pcl::PointCloud<pcl::PointXYZ>::Ptr sor = statistical_outlier_filter(pass_through);
    save_cloud("cloud_sor.vtk",sor);

    // Estimate normals
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals = estimate_normals(sor);

    // Construct mesh
    pcl::PolygonMesh triangles = greedy_projection_mesh_reconstruction(cloud_with_normals);

    // Save result to file
    pcl::io::saveVTKFile ("mesh2.vtk", triangles);

//    getchar();
    return triangles;
}