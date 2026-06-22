#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>

int main()
{
    try
    {
        // Configure RealSense pipeline
        rs2::pipeline pipe;
        rs2::config cfg;

        cfg.enable_stream(RS2_STREAM_COLOR,
                          640,
                          480,
                          RS2_FORMAT_BGR8,
                          30);

        pipe.start(cfg);

        // ArUco setup
        auto dictionary =
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

        // cv::aruco::DetectorParameters detectorParams;
        // cv::aruco::ArucoDetector detector(dictionary,
        //                                   detectorParams);

        cv::Ptr<cv::aruco::DetectorParameters> detectorParams =
            cv::aruco::DetectorParameters::create();

        while (true)
        {
            // Wait for a new frame
            rs2::frameset frames = pipe.wait_for_frames();

            rs2::video_frame color_frame =
                frames.get_color_frame();

            if (!color_frame)
                continue;

            // Convert RealSense frame to OpenCV Mat
            cv::Mat frame(
                cv::Size(color_frame.get_width(),
                         color_frame.get_height()),
                CV_8UC3,
                (void *)color_frame.get_data(),
                cv::Mat::AUTO_STEP);

            // Clone because RealSense reuses memory
            cv::Mat image = frame.clone();

            // Detect markers
            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f>> markerCorners;
            std::vector<std::vector<cv::Point2f>> rejectedCandidates;

            // detector.detectMarkers(image,
            //                        markerCorners,
            //                        markerIds,
            //                        rejectedCandidates);

            cv::aruco::detectMarkers(image,
                                     dictionary,
                                     markerCorners,
                                     markerIds,
                                     detectorParams,
                                     rejectedCandidates);

            if (!markerIds.empty())
            {
                cv::aruco::drawDetectedMarkers(image,
                                               markerCorners,
                                               markerIds);

                for (size_t i = 0; i < markerIds.size(); i++)
                {
                    std::cout << "Marker ID: "
                              << markerIds[i]
                              << std::endl;

                    for (int j = 0; j < 4; j++)
                    {
                        std::cout << "Corner "
                                  << j
                                  << ": "
                                  << markerCorners[i][j]
                                  << std::endl;
                    }

                    std::cout << "-----------------\n";
                }
            }

            cv::imshow("D455 ArUco Detection", image);

            char key = cv::waitKey(1);

            if (key == 27) // ESC
                break;
        }

        pipe.stop();
    }
    catch (const rs2::error &e)
    {
        std::cerr << "RealSense error: "
                  << e.what()
                  << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception: "
                  << e.what()
                  << std::endl;
    }

    return 0;
}