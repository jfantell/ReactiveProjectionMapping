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
#include <iostream>


using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using namespace std;

////Source: https://github.com/eMrazSVK/JetsonSLAM/blob/master/pcl_testing.cpp
//// Get RGB values based on normals - texcoords, normals value [u v]
//std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords)
//{
//    const int w = texture.get_width(), h = texture.get_height();
//
//    // convert normals [u v] to basic coords [x y]
//    int x = std::min(std::max(int(texcoords.u*w + .5f), 0), w - 1);
//    int y = std::min(std::max(int(texcoords.v*h + .5f), 0), h - 1);
//
//    int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
//    const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());
//    return std::tuple<uint8_t, uint8_t, uint8_t>(texture_data[idx], texture_data[idx+1], texture_data[idx+2]);
//}
//
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
    for (int i=0; i < points.size(); i++)
    {
        //XYZ values
        cloud->points[i].x = vertex_coordinates[i].x;
        cloud->points[i].y = vertex_coordinates[i].y;
        cloud->points[i].z = vertex_coordinates[i].z;

//        //Color coordinates
//        std::tuple<uint8_t, uint8_t, uint8_t> current_color;
//        current_color = get_texcolor(color, texture_coordinates[i]);
//
//        // Reversed order- 2-1-0 because of BGR model used in camera
//        cloud->points[i].r = std::get<2>(current_color);
//        cloud->points[i].g = std::get<1>(current_color);
//        cloud->points[i].b = std::get<0>(current_color);
    }

    return cloud;
}


const pcl::PointCloud<pcl::PointXYZ>::Ptr copy_XYZRGB_to_XYZ(const pcl_ptr& xyzrgb_cloud){
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    size_t size = xyzrgb_cloud->size();
    for(int i = 0 ; i < size ; i++){
        float x = xyzrgb_cloud->points[i].x;
        float y = xyzrgb_cloud->points[i].y;
        float z = xyzrgb_cloud->points[i].z;
        pcl::PointXYZ xyz_point = pcl::PointXYZ(x,y,z);
        xyz_cloud->push_back(xyz_point);
    }
//    for(int i = 0 ; i < size ; i++){
//        float x = xyz_cloud->points[i].x;
//        float y = xyz_cloud->points[i].y;
//        float z = xyz_cloud->points[i].z;
//        cout << "xyz " << x << y << z << endl;
//    }
    return xyz_cloud;
}

//Source: http://www.pointclouds.org/documentation/tutorials/greedy_projection.php
pcl::PolygonMesh create_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_xyz){
    cout << "Constructing Mesh" << endl;

    //Downsample point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_xyz);
    sor.setLeafSize (0.03f, 0.03f, 0.03f);
    sor.filter (*cloud_downsampled);
    cout << "Downsampled Cloud" << endl;

    //Save downsampled pointcloud
    pcl::PCLPointCloud2 cloud_blob_samp;
    pcl::toPCLPointCloud2 (*cloud_downsampled, cloud_blob_samp);
    pcl::io::saveVTKFile ("cloud_downsampled.vtk", cloud_blob_samp);

    // Apply pass through filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_downsampled);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.5);
    pass.filter (*cloud_filtered);
    cout << "Filtered Cloud" << endl;

    //Save filtered point cloud to file
    pcl::PCLPointCloud2 cloud_blob_filt;
    pcl::toPCLPointCloud2 (*cloud_filtered, cloud_blob_filt);
    pcl::io::saveVTKFile ("cloud_filtered.vtk", cloud_blob_filt);

    //Upsample
    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_upsample (new pcl::search::KdTree<pcl::PointXYZ>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points (new pcl::PointCloud<pcl::PointNormal> );

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setComputeNormals (true);

    // Set parameters
    mls.setInputCloud (cloud_filtered);
    mls.setPolynomialOrder (2);
    mls.setSearchMethod (tree_upsample);
    mls.setSearchRadius (0.03);

    // Reconstruct
    mls.process (*mls_points);


    // Normal estimation*
//    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
//    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//    tree->setInputCloud (cloud_filtered);
//    n.setInputCloud (cloud_filtered);
//    n.setSearchMethod (tree);
//    n.setKSearch (50);
//    n.compute (*normals);

//    // Concatenate the XYZ and normal fields*
//    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
//    pcl::concatenateFields (*cloud_filtered, *normals, *cloud_with_normals);
//    cout << "Created and Concatenated Normals with Vertex Coordinates " << endl;
//
//    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree_mesh (new pcl::search::KdTree<pcl::PointNormal>);
    tree_mesh->setInputCloud(mls_points);

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
    gp3.setInputCloud (mls_points);
    gp3.setSearchMethod (tree_mesh);
    gp3.reconstruct (triangles);

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();
    cout << "Mesh Constructed" << endl;

    // Save result to file
    pcl::io::saveVTKFile ("mesh.vtk", triangles);

//    getchar();
    return triangles;
}