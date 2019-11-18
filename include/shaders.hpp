//
// This file includes the logic necessary to:
//  - Create a point cloud from a RealSense depth frame
//  - Map texture coordinates to the point cloud
//  - Extract object and texture coordinates from the point cloud
//  - Connect application data to the vertex and fragment shaders
//


#ifndef shaders_hpp
#define shaders_hpp

#include <GLFW/glfw3.h>
#include <librealsense2/rs.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

void init(GLFWwindow* window, const char * vertShaderFile, const char * fragShderFile, int textureMode_, const char * textureImage);
void display(GLFWwindow* window, double currentTime, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
void window_size_callback(GLFWwindow* win, int newWidth, int newHeight);

#endif /* shaders_hpp */
