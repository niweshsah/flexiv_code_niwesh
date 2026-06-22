#include <flexiv/rdk/robot.hpp>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <array>
#include <vector>

// Convert Flexiv flange pose [x,y,z,qw,qx,qy,qz]
// into a 4x4 homogeneous transformation matrix
cv::Mat flexivPoseToMatrix(const std::array<double, 7>& pose)
{
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);

    double x  = pose[0];
    double y  = pose[1];
    double z  = pose[2];

    double qw = pose[3];
    double qx = pose[4];
    double qy = pose[5];
    double qz = pose[6];

    // Rotation matrix from quaternion
    T.at<double>(0, 0) = 1.0 - 2.0 * (qy * qy + qz * qz);
    T.at<double>(0, 1) = 2.0 * (qx * qy - qz * qw);
    T.at<double>(0, 2) = 2.0 * (qx * qz + qy * qw);

    T.at<double>(1, 0) = 2.0 * (qx * qy + qz * qw);
    T.at<double>(1, 1) = 1.0 - 2.0 * (qx * qx + qz * qz);
    T.at<double>(1, 2) = 2.0 * (qy * qz - qx * qw);

    T.at<double>(2, 0) = 2.0 * (qx * qz - qy * qw);
    T.at<double>(2, 1) = 2.0 * (qy * qz + qx * qw);
    T.at<double>(2, 2) = 1.0 - 2.0 * (qx * qx + qy * qy);

    T.at<double>(0, 3) = x;
    T.at<double>(1, 3) = y;
    T.at<double>(2, 3) = z;

    return T;
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: ./find_red_base <robot_sn>\n";
        return -1;
    }

    try {

        //----------------------------------------------------------
        // Connect to Flexiv
        //----------------------------------------------------------
        std::cout << "[Robot] Connecting to Flexiv: "
                  << argv[1] << std::endl;

        flexiv::rdk::Robot robot(argv[1]);

        std::cout << "[Robot] Connected" << std::endl;

        //----------------------------------------------------------
        // Start RealSense
        //----------------------------------------------------------
        std::cout << "[Vision] Starting RealSense..."
                  << std::endl;

        rs2::pipeline pipe;

        rs2::pipeline_profile profile =
            pipe.start();

        rs2::align align_to_color(
            RS2_STREAM_COLOR);

        auto color_stream =
            profile.get_stream(RS2_STREAM_COLOR)
                   .as<rs2::video_stream_profile>();

        rs2_intrinsics intrinsics =
            color_stream.get_intrinsics();

        //----------------------------------------------------------
        // Hand-eye calibration matrix
        //
        // Replace with actual calibration values
        //----------------------------------------------------------
        cv::Mat T_flange_camera =
            cv::Mat::eye(4, 4, CV_64F);

        T_flange_camera.at<double>(0, 3) = 0.05;
        T_flange_camera.at<double>(1, 3) = 0.00;
        T_flange_camera.at<double>(2, 3) = 0.08;

        //----------------------------------------------------------
        // Windows
        //----------------------------------------------------------
        cv::namedWindow(
            "Base Frame Tracking",
            cv::WINDOW_AUTOSIZE);

        cv::namedWindow(
            "Red Mask",
            cv::WINDOW_AUTOSIZE);

        //----------------------------------------------------------
        // Main loop
        //----------------------------------------------------------
        while (true) {

            rs2::frameset frames =
                pipe.wait_for_frames();

            frames =
                align_to_color.process(frames);

            rs2::video_frame color_frame =
                frames.get_color_frame();

            rs2::depth_frame depth_frame =
                frames.get_depth_frame();

            if (!color_frame || !depth_frame)
                continue;

            //------------------------------------------------------
            // Convert RealSense frame to OpenCV
            //------------------------------------------------------
            cv::Mat rs_image(
                cv::Size(
                    color_frame.get_width(),
                    color_frame.get_height()),
                CV_8UC3,
                (void*)color_frame.get_data(),
                cv::Mat::AUTO_STEP);

            cv::Mat color =
                rs_image.clone();

            cv::cvtColor(
                color,
                color,
                cv::COLOR_RGB2BGR);

            //------------------------------------------------------
            // Detect red object
            //------------------------------------------------------
            cv::Mat hsv;
            cv::Mat mask1;
            cv::Mat mask2;
            cv::Mat mask;

            cv::cvtColor(
                color,
                hsv,
                cv::COLOR_BGR2HSV);

            // Lower red range
            cv::inRange(
                hsv,
                cv::Scalar(0, 100, 100),
                cv::Scalar(10, 255, 255),
                mask1);

            // Upper red range
            cv::inRange(
                hsv,
                cv::Scalar(170, 100, 100),
                cv::Scalar(180, 255, 255),
                mask2);

            mask = mask1 | mask2;

            //------------------------------------------------------
            // Morphological filtering
            //------------------------------------------------------
            cv::Mat kernel =
                cv::getStructuringElement(
                    cv::MORPH_ELLIPSE,
                    cv::Size(5, 5));

            cv::morphologyEx(
                mask,
                mask,
                cv::MORPH_OPEN,
                kernel);

            cv::morphologyEx(
                mask,
                mask,
                cv::MORPH_CLOSE,
                kernel);

            //------------------------------------------------------
            // Find contours
            //------------------------------------------------------
            std::vector<std::vector<cv::Point>>
                contours;

            cv::findContours(
                mask,
                contours,
                cv::RETR_EXTERNAL,
                cv::CHAIN_APPROX_SIMPLE);

            double max_area = 0.0;
            int largest_idx = -1;

            for (size_t i = 0;
                 i < contours.size();
                 ++i) {

                double area =
                    cv::contourArea(
                        contours[i]);

                if (area > max_area) {

                    max_area = area;
                    largest_idx =
                        static_cast<int>(i);
                }
            }

            //------------------------------------------------------
            // Process largest red object
            //------------------------------------------------------
            if (largest_idx != -1 &&
                max_area > 500.0) {

                cv::Rect bbox =
                    cv::boundingRect(
                        contours[largest_idx]);

                cv::rectangle(
                    color,
                    bbox,
                    cv::Scalar(0, 0, 255),
                    2);

                cv::Moments M =
                    cv::moments(
                        contours[largest_idx]);

                if (M.m00 > 0) {

                    int cx =
                        static_cast<int>(
                            M.m10 / M.m00);

                    int cy =
                        static_cast<int>(
                            M.m01 / M.m00);

                    //--------------------------------------------------
                    // Get depth
                    //--------------------------------------------------
                    float depth =
                        depth_frame.get_distance(
                            cx,
                            cy);

                    if (depth > 0.01f) {

                        //--------------------------------------------------
                        // Pixel -> Camera frame
                        //--------------------------------------------------
                        float pixel[2] = {
                            static_cast<float>(cx),
                            static_cast<float>(cy)
                        };

                        float point_camera[3];

                        rs2_deproject_pixel_to_point(
                            point_camera,
                            &intrinsics,
                            pixel,
                            depth);

                        //--------------------------------------------------
                        // Camera -> Homogeneous
                        //--------------------------------------------------
                        cv::Mat P_camera =
                            (cv::Mat_<double>(4,1)
                             << point_camera[0],
                                point_camera[1],
                                point_camera[2],
                                1.0);

                        //--------------------------------------------------
                        // Flange pose
                        //--------------------------------------------------
                        auto state =
                            robot.states();

                        cv::Mat T_base_flange =
                            flexivPoseToMatrix(
                                state.flange_pose);

                        //--------------------------------------------------
                        // Camera -> Base
                        //--------------------------------------------------
                        cv::Mat P_base =
                            T_base_flange *
                            T_flange_camera *
                            P_camera;

                        double bx =
                            P_base.at<double>(0);

                        double by =
                            P_base.at<double>(1);

                        double bz =
                            P_base.at<double>(2);

                        //--------------------------------------------------
                        // Print coordinates
                        //--------------------------------------------------
                        std::cout
                            << "Target in BASE frame: "
                            << "X = "
                            << bx
                            << " m, Y = "
                            << by
                            << " m, Z = "
                            << bz
                            << " m"
                            << std::endl;

                        //--------------------------------------------------
                        // Visualization
                        //--------------------------------------------------
                        cv::circle(
                            color,
                            cv::Point(cx, cy),
                            5,
                            cv::Scalar(
                                255,
                                0,
                                0),
                            -1);

                        char text[150];

                        snprintf(
                            text,
                            sizeof(text),
                            "Base: %.3f %.3f %.3f",
                            bx,
                            by,
                            bz);

                        cv::putText(
                            color,
                            text,
                            cv::Point(
                                20,
                                40),
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.7,
                            cv::Scalar(
                                0,
                                255,
                                0),
                            2);
                    }
                }
            }

            //------------------------------------------------------
            // Display
            //------------------------------------------------------
            cv::imshow(
                "Base Frame Tracking",
                color);

            cv::imshow(
                "Red Mask",
                mask);

            int key =
                cv::waitKey(1);

            if ((key & 0xFF) == 27)
                break;
        }

        //----------------------------------------------------------
        // Cleanup
        //----------------------------------------------------------
        pipe.stop();

        cv::destroyAllWindows();
    }
    catch (const std::exception& e) {

        std::cerr
            << "Error: "
            << e.what()
            << std::endl;

        return -1;
    }

    return 0;
}