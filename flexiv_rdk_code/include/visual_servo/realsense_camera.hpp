#pragma once

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

class RealSenseCamera {
public:
    // Constructor handles initialization and auto-negotiation
    RealSenseCamera();
    
    // Destructor safely stops the camera
    ~RealSenseCamera();

    // Grabs the latest frames and converts them to OpenCV format
    // Returns true if successful, false if frames were dropped
    bool get_frames(cv::Mat& color_mat, cv::Mat& depth_mat);

private:
    rs2::pipeline pipe;
    rs2::align align_to_color;
    rs2::colorizer color_map;
};