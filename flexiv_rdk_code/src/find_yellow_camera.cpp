#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>

int main() {
    try {
        std::cout << "[Vision] Starting RealSense Pipeline..." << std::endl;

        rs2::pipeline pipe;
        rs2::pipeline_profile profile = pipe.start();

        rs2::align align_to_color(RS2_STREAM_COLOR);

        // Camera intrinsics
        auto stream =
            profile.get_stream(RS2_STREAM_COLOR)
                .as<rs2::video_stream_profile>();

        rs2_intrinsics intrinsics = stream.get_intrinsics();

        cv::namedWindow("Camera Frame Tracking", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("Red Mask", cv::WINDOW_AUTOSIZE);

        while (true) {
            rs2::frameset frames = pipe.wait_for_frames();
            frames = align_to_color.process(frames);

            rs2::video_frame color_frame = frames.get_color_frame();
            rs2::depth_frame depth_frame = frames.get_depth_frame();

            if (!color_frame || !depth_frame)
                continue;

            //---------------------------------------------------------
            // Convert RealSense frame to OpenCV Mat
            //---------------------------------------------------------
            cv::Mat color_mat(
                cv::Size(color_frame.get_width(),
                         color_frame.get_height()),
                CV_8UC3,
                (void*)color_frame.get_data(),
                cv::Mat::AUTO_STEP);

            // Clone so that OpenCV modifications do not affect RS buffer
            cv::Mat color = color_mat.clone();

            // RealSense gives RGB
            cv::cvtColor(color, color, cv::COLOR_RGB2BGR);

            //---------------------------------------------------------
            // RED COLOR DETECTION
            //---------------------------------------------------------
            cv::Mat hsv, mask1, mask2, mask;

            cv::cvtColor(color, hsv, cv::COLOR_BGR2HSV);

            // Lower red
            cv::inRange(
                hsv,
                cv::Scalar(0, 100, 100),
                cv::Scalar(10, 255, 255),
                mask1);

            // Upper red
            cv::inRange(
                hsv,
                cv::Scalar(170, 100, 100),
                cv::Scalar(180, 255, 255),
                mask2);

            mask = mask1 | mask2;

            //---------------------------------------------------------
            // Remove noise
            //---------------------------------------------------------
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

            //---------------------------------------------------------
            // Find contours
            //---------------------------------------------------------
            std::vector<std::vector<cv::Point>> contours;

            cv::findContours(
                mask,
                contours,
                cv::RETR_EXTERNAL,
                cv::CHAIN_APPROX_SIMPLE);

            double max_area = 0.0;
            int largest_idx = -1;

            for (size_t i = 0; i < contours.size(); i++) {
                double area = cv::contourArea(contours[i]);

                if (area > max_area) {
                    max_area = area;
                    largest_idx = i;
                }
            }

            //---------------------------------------------------------
            // Process largest red object
            //---------------------------------------------------------
            if (largest_idx != -1 && max_area > 500.0) {

                cv::Rect bbox =
                    cv::boundingRect(contours[largest_idx]);

                cv::rectangle(
                    color,
                    bbox,
                    cv::Scalar(0, 0, 255),
                    2);

                cv::Moments M =
                    cv::moments(contours[largest_idx]);

                if (M.m00 > 0) {

                    int cx =
                        static_cast<int>(M.m10 / M.m00);

                    int cy =
                        static_cast<int>(M.m01 / M.m00);

                    //-------------------------------------------------
                    // Obtain depth
                    //-------------------------------------------------
                    float depth_meters =
                        depth_frame.get_distance(cx, cy);

                    // Search nearby if depth is invalid
                    if (depth_meters <= 0.0f) {

                        for (int dy = -2; dy <= 2; dy++) {
                            for (int dx = -2; dx <= 2; dx++) {

                                int x = cx + dx;
                                int y = cy + dy;

                                if (x < 0 ||
                                    x >= depth_frame.get_width() ||
                                    y < 0 ||
                                    y >= depth_frame.get_height())
                                    continue;

                                depth_meters =
                                    depth_frame.get_distance(x, y);

                                if (depth_meters > 0.0f)
                                    break;
                            }

                            if (depth_meters > 0.0f)
                                break;
                        }
                    }

                    //-------------------------------------------------
                    // 2D → 3D conversion
                    //-------------------------------------------------
                    if (depth_meters > 0.0f) {

                        float pixel[2] = {
                            static_cast<float>(cx),
                            static_cast<float>(cy)
                        };

                        float point[3];

                        rs2_deproject_pixel_to_point(
                            point,
                            &intrinsics,
                            pixel,
                            depth_meters);

                        std::cout
                            << "Target (Camera Frame): "
                            << "X = " << point[0] << " m, "
                            << "Y = " << point[1] << " m, "
                            << "Z = " << point[2] << " m"
                            << std::endl;

                        //-------------------------------------------------
                        // Visualization
                        //-------------------------------------------------
                        cv::circle(
                            color,
                            cv::Point(cx, cy),
                            5,
                            cv::Scalar(255, 0, 0),
                            -1);

                        char text[100];

                        std::snprintf(
                            text,
                            sizeof(text),
                            "X: %.2f  Y: %.2f  Z: %.2f m",
                            point[0],
                            point[1],
                            point[2]);

                        cv::putText(
                            color,
                            text,
                            cv::Point(cx - 100, cy - 20),
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.6,
                            cv::Scalar(0, 255, 0),
                            2);
                    }
                }
            }

            //---------------------------------------------------------
            // Display
            //---------------------------------------------------------
            cv::imshow("Camera Frame Tracking", color);
            cv::imshow("Red Mask", mask);

            int key = cv::waitKey(1);

            if ((key & 0xFF) == 27) // ESC
                break;
        }

        pipe.stop();
        cv::destroyAllWindows();

    }
    catch (const rs2::error& e) {
        std::cerr << "RealSense Error: "
                  << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (const std::exception& e) {
        std::cerr << "Exception: "
                  << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}