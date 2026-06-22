// #include "realsense_camera.hpp"
#include <visual_servo/realsense_camera.hpp>
#include <iostream>
#include <vector>

int main()
{
    try
    {
        // 1. Initialize the camera class
        RealSenseCamera camera;

        cv::namedWindow("red Tracker", cv::WINDOW_AUTOSIZE); // create a window with name red tracker
        cv::namedWindow("red Mask", cv::WINDOW_AUTOSIZE);

        std::cout << "Starting tracking loop. Press ESC to exit." << std::endl;

        // 2. Main Tracking Loop
        while (true)
        {
            cv::Mat color, depth;

            // Call our new function to get the frames
            if (!camera.get_frames(color, depth))
            {
                continue; // Skip iteration if frames dropped
            }

            // 3. Image Processing: Convert BGR to HSV color space
            // 3. Image Processing: Convert BGR to HSV color space
            cv::Mat hsv, mask, mask1, mask2;
            cv::cvtColor(color, hsv, cv::COLOR_BGR2HSV);

            // Lower red range
            cv::inRange(hsv,
                        cv::Scalar(0, 100, 100),
                        cv::Scalar(10, 255, 255),
                        mask1);

            // Upper red range
            cv::inRange(hsv,
                        cv::Scalar(170, 100, 100),
                        cv::Scalar(180, 255, 255),
                        mask2);

            // Combine masks
            mask = mask1 | mask2;

            // Remove noise
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
                             cv::Mat(), cv::Point(-1, -1), 2);

            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE,
                             cv::Mat(), cv::Point(-1, -1), 2);

            // 6. Contour Detection
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            // 7. Draw bounding boxes around detected red objects
            for (const auto &contour : contours)
            {
                double area = cv::contourArea(contour);

                // Filter out tiny artifacts
                if (area > 500.0)
                {
                    cv::Rect bounding_box = cv::boundingRect(contour);

                    // Draw the red box
                    cv::rectangle(color, bounding_box, cv::Scalar(0, 255, 255), 2);

                    // Calculate and draw the center point (centroid)
                    cv::Moments M = cv::moments(contour);
                    if (M.m00 > 0)
                    {
                        int cx = int(M.m10 / M.m00);
                        int cy = int(M.m01 / M.m00);
                        // Draw a red dot in the center
                        cv::circle(color, cv::Point(cx, cy), 5, cv::Scalar(0, 0, 255), -1);
                    }
                }
            }

            // 8. Display results
            cv::imshow("red Tracker", color);
            cv::imshow("red Mask", mask); // Helpful for tuning HSV values

            if ((cv::waitKey(1) & 0xFF) == 27)
                break; // Exit on ESC
        }
    }
    catch (const rs2::error &e)
    {
        std::cerr << "RealSense error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}