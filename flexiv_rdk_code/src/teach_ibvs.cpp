#include <flexiv/rdk/robot.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <array>

// Print Flexiv pose
void printPose(const std::array<double, 7>& pose,
               const std::string& label)
{
    std::cout << label << " [x y z qw qx qy qz]: ";

    for (double val : pose) {
        std::cout << std::fixed
                  << std::setprecision(4)
                  << val << " ";
    }

    std::cout << std::endl;
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0]
                  << " <robot_ip>" << std::endl;
        return -1;
    }

    try {
        /* ---------------- Robot Initialization ---------------- */
        flexiv::rdk::Robot robot(argv[1]);

        std::cout << "Enabling robot..." << std::endl;
        robot.Enable();

        while (!robot.operational()) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(500));
        }

        std::cout << "Robot is operational." << std::endl;

        /* ---------------- RealSense Initialization ---------------- */
        rs2::pipeline pipe;
        rs2::config cfg;

        cfg.enable_stream(
            RS2_STREAM_COLOR,
            640,
            480,
            RS2_FORMAT_RGB8,
            30);

        pipe.start(cfg);

        std::cout << "\n--- TEACH MODE ---" << std::endl;
        std::cout << "Place the object in view." << std::endl;
        std::cout << "Press 's' to save pose." << std::endl;
        std::cout << "Press ESC to quit.\n" << std::endl;

        /* ---------------- Main Loop ---------------- */
        while (true) {

            rs2::frameset frames = pipe.wait_for_frames();

            rs2::video_frame color_frame =
                frames.get_color_frame();

            if (!color_frame) {
                continue;
            }

            cv::Mat color(
                cv::Size(640, 480),
                CV_8UC3,
                (void*)color_frame.get_data(),
                cv::Mat::AUTO_STEP);

            color = color.clone();

            cv::cvtColor(
                color,
                color,
                cv::COLOR_RGB2BGR);

            /* ---------- Red Object Detection ---------- */

            cv::Mat hsv;
            cv::cvtColor(
                color,
                hsv,
                cv::COLOR_BGR2HSV);

            cv::Mat mask1, mask2, mask;

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

            cv::erode(
                mask,
                mask,
                cv::Mat(),
                cv::Point(-1, -1),
                1);

            cv::dilate(
                mask,
                mask,
                cv::Mat(),
                cv::Point(-1, -1),
                2);

            std::vector<std::vector<cv::Point>> contours;

            cv::findContours(
                mask,
                contours,
                cv::RETR_EXTERNAL,
                cv::CHAIN_APPROX_SIMPLE);

            bool object_detected = false;
            cv::Rect box;

            if (!contours.empty()) {

                auto largest =
                    std::max_element(
                        contours.begin(),
                        contours.end(),
                        [](const auto& a,
                           const auto& b)
                        {
                            return cv::contourArea(a)
                                 < cv::contourArea(b);
                        });

                double area =
                    cv::contourArea(*largest);

                if (area > 500.0) {

                    object_detected = true;

                    box = cv::boundingRect(*largest);

                    cv::rectangle(
                        color,
                        box,
                        cv::Scalar(0, 255, 0),
                        2);

                    cv::putText(
                        color,
                        "Red Object",
                        cv::Point(box.x,
                                  box.y - 10),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.6,
                        cv::Scalar(0, 255, 0),
                        2);
                }
            }

            /* ---------- Display ---------- */

            cv::imshow(
                "Teach Mode",
                color);

            int key = cv::waitKey(1) & 0xFF;

            /* ---------- Save Teaching Data ---------- */

            if (key == 's' &&
                object_detected) {

                auto state =
                    robot.states();

                std::cout
                    << "\n>>> TEACHING DATA CAPTURED <<<"
                    << std::endl;

                printPose(
                    state.flange_pose,
                    "Current Flange Pose:");

                std::cout
                    << "Bounding Box [x, y, w, h]: "
                    << box.x << ", "
                    << box.y << ", "
                    << box.width << ", "
                    << box.height
                    << std::endl;

                std::cout
                    << "--------------------------------"
                    << std::endl;
            }

            /* ---------- Exit ---------- */

            if (key == 27) {
                break;
            }
        }

        /* ---------------- Cleanup ---------------- */

        pipe.stop();
        cv::destroyAllWindows();
    }
    // catch (const flexiv::rdk::Exception& e) {

    //     std::cerr
    //         << "Flexiv exception: "
    //         << e.what()
    //         << std::endl;

    //     return -1;
    // }
    catch (const rs2::error& e) {

        std::cerr
            << "RealSense error: "
            << e.what()
            << std::endl;

        return -1;
    }
    catch (const std::exception& e) {

        std::cerr
            << "Exception: "
            << e.what()
            << std::endl;

        return -1;
    }

    return 0;
}