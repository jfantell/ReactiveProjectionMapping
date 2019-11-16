//
// Created by hayley on 11/7/19.
//

#include "frame_processing.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <unistd.h>

#include <string>
using namespace cv;
using namespace std;

// Convert rs2::frame to cv::Mat
//Source: Intel RealSense2 SDK
cv::Mat frame_to_mat(rs2::frame &f)
{
    using namespace cv;
    using namespace rs2;

    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();
    //Should always be first option
    if (f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        Mat r = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
        return r;
    }
    else if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        Mat r = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
        cvtColor(r, r, COLOR_RGB2BGR);
//        imshow("Depth",r);
//        waitKey(1);
        return r;
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32)
    {
        return Mat(Size(w, h), CV_32FC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
}



Mat process_frame_image( Mat & image )
{
//    cout << "Frame size and depth " << image.size() << " " << image.channels() << endl;
    Mat final_image;

    //METHOD #1: Split channels, adjust each channel individually, merge channels
//    Mat bgr[3];
//    split(image,bgr);
//    bgr[0].setTo(255,bgr[0] > 127);
//    bgr[1].setTo(255,bgr[1] > 127);
//    bgr[2].setTo(255,bgr[2] > 70);
//    vector<Mat> channels = {bgr[0],bgr[1],bgr[2]};
//    merge(channels, final_image);

    //METHOD #2: Canny Edge Detection
//    final_image = image;
//    Mat gray_image, edge_image, kernel;
//    cvtColor(image,gray_image,CV_BGR2GRAY);
//    Canny(gray_image,edge_image,150,200, 3, true);
//    final_image.setTo(Scalar(255,114,51),edge_image==255);

    //METHOD #3-7: Colorspace Conversions
   // cvtColor(image, final_image, CV_BGR2RGB);
    cvtColor(image, final_image, CV_RGB2HSV);
    //cvtColor(image, final_image, CV_BGR2Lab);
//    cvtColor(image, final_image, CV_BGR2HLS);

    //Display modified texture image
    imshow("OpenGL Input Texture",final_image);
    waitKey(1);

    return final_image;
}




