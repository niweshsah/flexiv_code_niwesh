#include <visual_servo/realsense_camera.hpp>
#include <iostream>
#include <thread>
#include <chrono> 

RealSenseCamera::RealSenseCamera() : align_to_color(RS2_STREAM_COLOR) {
    std::cout << "[Camera] Attempting to connect to RealSense D455..." << std::endl;
    pipe.start(); // Auto-negotiate resolution
    std::cout << "[Camera] Successfully connected and streaming!" << std::endl;
}

RealSenseCamera::~RealSenseCamera() {
    try {
        pipe.stop();
        std::cout << "[Camera] Pipeline stopped safely." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[Camera] Error during destruction: " << e.what() << std::endl;
    }
}

bool RealSenseCamera::get_frames(cv::Mat& color_mat, cv::Mat& depth_mat) {
    try {
        // Block until the next set of frames arrives
        rs2::frameset frames = pipe.wait_for_frames();

        // Align depth to color
        frames = align_to_color.process(frames);

        rs2::video_frame color_frame = frames.get_color_frame();
        rs2::depth_frame depth_frame = frames.get_depth_frame();

        if (!color_frame || !depth_frame) return false;

        // Colorize depth for visualization
        rs2::video_frame depth_colorized = depth_frame.apply_filter(color_map);

        const int w = color_frame.get_width();
        const int h = color_frame.get_height();

        // Create temporary wrappers around the RealSense memory buffers (zero-copy)
        cv::Mat temp_color(cv::Size(w, h), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat temp_depth(cv::Size(w, h), CV_8UC3, (void*)depth_colorized.get_data(), cv::Mat::AUTO_STEP);

        // Convert color spectrum and deep-copy frames to ensure persistence
        cv::cvtColor(temp_color, color_mat, cv::COLOR_RGB2BGR);
        depth_mat = temp_depth.clone(); 

        return true;
    } catch (const std::exception& e) {
        std::cerr << "[Camera] Frame acquisition failed: " << e.what() << std::endl;
        return false;
    }
}

// ============================================================================
// STANDALONE TEST HARNESS
// This code only exists when the target is explicitly built as an executable.
// ============================================================================
#ifdef BUILD_STANDALONE
int main(int argc, char* argv[]) {
    try {
        RealSenseCamera camera;
        
        const std::string window_name = "RealSense Camera Diagnostic (Press ESC to exit)";
        cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

        std::cout << "\n-----------------------------------------------------" << std::endl;
        std::cout << "Diagnostic started. Focus the video window to interact." << std::endl;
        std::cout << "-----------------------------------------------------\n" << std::endl;

        cv::Mat color_feed, depth_feed;
        cv::Mat diagnostic_canvas;

        while (true) {
            if (!camera.get_frames(color_feed, depth_feed)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            // Stack color and heatmap depth arrays horizontally for simultaneous observation
            cv::hconcat(color_feed, depth_feed, diagnostic_canvas);

            // Print telemetry info directly on frame canvas
            std::string resolution_str = "Res: " + std::to_string(color_feed.cols) + "x" + std::to_string(color_feed.rows);
            cv::putText(diagnostic_canvas, resolution_str, cv::Point(20, 40), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

            cv::imshow(window_name, diagnostic_canvas);

            // Trap escape sequence (ASCII code 27)
            if ((cv::waitKey(1) & 0xFF) == 27) {
                std::cout << "[Diagnostic] Termination sequence detected." << std::endl;
                break;
            }
            
            // Fault check: Break out safely if user closed the canvas manually via window manager
            if (cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) < 0) {
                break;
            }
        }
        
        cv::destroyAllWindows();
    } catch (const rs2::error& e) {
        std::cerr << "[Diagnostic Critical] API call failed: " << e.what() << std::endl;
        return EXIT_FAILURE;
    } catch (const std::exception& e) {
        std::cerr << "[Diagnostic Critical] Runtime exception: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
#endif