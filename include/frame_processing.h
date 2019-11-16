//
// Created by hayley on 11/7/19.
//

#ifndef REALSENSE2_FRAME_PROCESSING_H
#define REALSENSE2_FRAME_PROCESSING_H

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

cv::Mat frame_to_mat(rs2::frame &f);
cv::Mat process_frame_image( cv::Mat & image );

#endif //REALSENSE2_FRAME_PROCESSING_H
