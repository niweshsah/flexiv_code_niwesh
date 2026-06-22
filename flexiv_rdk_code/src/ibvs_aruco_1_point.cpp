
#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/model.hpp>
#include <memory>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>

#include <visp3/vs/vpServo.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpPoseVector.h>
#include <visp3/core/vpQuaternionVector.h>

#include <iostream>
#include <array>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <opencv2/aruco.hpp>

//----------------------------------------------------------
// Thread-Safe Shared Data Structure (1 Point)
//----------------------------------------------------------
struct SharedVisionData
{
    std::mutex mtx;
    bool target_found = false;
    double center_x_norm = 0.0;
    double center_y_norm = 0.0;
    double Z_depth = 1.0;
    double safety_damping = 1.0; // <-- NEW
};

//----------------------------------------------------------
// Helper Functions
//----------------------------------------------------------
cv::Mat flexivPoseToMatrix(const std::array<double, 7> &pose)
{
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    double x = pose[0], y = pose[1], z = pose[2];
    double qw = pose[3], qx = pose[4], qy = pose[5], qz = pose[6];

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

double clamp(double val, double min_val, double max_val)
{
    return std::max(min_val, std::min(val, max_val));
}

//----------------------------------------------------------
// Main IBVS Application Class
//----------------------------------------------------------
class VisualServoApp
{
public:
    VisualServoApp(const std::string &robot_sn) : m_robot(robot_sn), running(true)
    {
        std::cout << "[INFO] [Init] Enabling robot and waiting for operational state..." << std::endl;
        m_robot.Enable();
        while (!m_robot.operational())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        std::cout << "[INFO] [Init] Initializing Robot Dynamics Model..." << std::endl;
        m_model = std::make_unique<flexiv::rdk::Model>(m_robot);

        std::cout << "[INFO] [Init] Configuring D455 Camera Intrinsics..." << std::endl;

        // --- HARDWARE SPECIFIC UPDATE: RealSense D455 Color Intrinsics (640x480) ---
        double px = 386.014465;
        double py = 385.656402;
        double u0 = 325.995117;
        double v0 = 244.844482;
        m_cam.initPersProjWithoutDistortion(px, py, u0, v0);

        // --- Flange to Camera Transform ---
        double tx = -0.125, ty = 0.0, tz = -0.035;
        m_fMc.buildFrom(tx, ty, tz, 0.0, 0.0, -M_PI / 2.0);

        std::cout << "[INFO] [Init] Switching to RT_CARTESIAN_MOTION_FORCE mode..." << std::endl;
        m_robot.SwitchMode(flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE);
    }

    ~VisualServoApp()
    {
        running = false;
        if (control_thread.joinable())
            control_thread.join();
        if (monitor_thread.joinable())
            monitor_thread.join();
        m_robot.Stop();
    }

    void startThreads()
    {
        control_thread = std::thread(&VisualServoApp::controlLoop, this);
        monitor_thread = std::thread(&VisualServoApp::monitorLoop, this); // <-- Add this
    }

    void updateVisionData(bool found, double cx = 0, double cy = 0, double depth = 0)
    {
        std::lock_guard<std::mutex> lock(m_shared.mtx);
        m_shared.target_found = found;
        if (found)
        {
            double cx_norm, cy_norm;
            vpPixelMeterConversion::convertPoint(m_cam, cx, cy, cx_norm, cy_norm);
            m_shared.center_x_norm = cx_norm;
            m_shared.center_y_norm = cy_norm;
            m_shared.Z_depth = depth;
        }
    }

private:
    void monitorLoop()
    {
        std::cout << "[INFO] [Monitor] 50Hz Safety Monitor Started." << std::endl;
        while (running)
        {
            auto current_states = m_robot.states();
            m_model->Update(current_states.q, current_states.dtheta);

            auto scores = m_model->configuration_score();
            double trans_score = scores.first;

            double calc_damping = 1.0;
            if (trans_score < 30.0)
            {
                calc_damping = std::max(0.1, trans_score / 30.0);
            }

            // Safely push the calculated damping to the shared struct
            {
                std::lock_guard<std::mutex> lock(m_shared.mtx);
                m_shared.safety_damping = calc_damping;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 50Hz
        }
    }

    void controlLoop()
    {
        std::cout << "[INFO] [Control] 1kHz Servo Loop Started." << std::endl;
        double dt = 0.001;
        uint64_t tick = 0;
        auto next_period = std::chrono::high_resolution_clock::now();
        const auto period = std::chrono::microseconds(1000);

        // --- THE FIX: Read physical state ONCE before the loop starts ---
        auto initial_state = m_robot.states();
        cv::Mat T_init = flexivPoseToMatrix(initial_state.flange_pose);
        vpHomogeneousMatrix bMf_cmd; // This will track our ideal software target
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                bMf_cmd[i][j] = T_init.at<double>(i, j);
            }
        }

        while (running)
        {
            next_period += period;
            tick++;

            vpColVector v_c(6, 0.0);
            bool valid_target = false;
            double current_x = 0, current_y = 0, current_z = 0;

            {
                std::lock_guard<std::mutex> lock(m_shared.mtx);
                if (m_shared.target_found)
                {
                    current_x = m_shared.center_x_norm;
                    current_y = m_shared.center_y_norm;
                    current_z = m_shared.Z_depth;
                    valid_target = true;
                }
            }

            if (valid_target)
            {

                // 4. Calculate damping: If score drops below 30 (Medium/Poor), start slowing down
                double damping = 1.0;

                {
                    std::lock_guard<std::mutex> lock(m_shared.mtx);
                    if (m_shared.target_found)
                    {
                        current_x = m_shared.center_x_norm;
                        current_y = m_shared.center_y_norm;
                        current_z = m_shared.Z_depth;
                        valid_target = true;
                    }
                    damping = m_shared.safety_damping; // <-- Grab the latest pre-calculated damping
                }

                double Z_desired = 0.4; // 30 cm desired height
                double lambda_xy = 2.5; // Gain for X/Y tracking speed
                double lambda_z = 1.0;  // Gain for Z depth speed

                // Use this damping factor on your velocity commands:
                v_c[0] = damping * (lambda_xy * current_x * current_z);
                v_c[1] = damping * (lambda_xy * current_y * current_z);
                v_c[2] = damping * (lambda_z * (current_z - Z_desired));

                // // 1. Calculate velocities to drive the normalized center to (0,0)
                // // Math: Vx = lambda * x * Z
                // v_c[0] = lambda_xy * current_x * current_z;
                // v_c[1] = lambda_xy * current_y * current_z;

                // // 2. Calculate velocity to drive current depth to desired depth
                // v_c[2] = lambda_z * (current_z - Z_desired);

                // 3. STRICTLY LOCK ALL ROTATIONS TO ZERO
                v_c[3] = 0.0; // Roll
                v_c[4] = 0.0; // Pitch
                v_c[5] = 0.0; // Yaw

                // 4. Apply safety limits (increased to 30 cm/s for smooth tracking)
                v_c[0] = clamp(v_c[0], -0.3, 0.3);
                v_c[1] = clamp(v_c[1], -0.3, 0.3);
                v_c[2] = clamp(v_c[2], -0.3, 0.3);
            }
            else
            {
                // Smooth velocity decay if target is temporarily lost
                for (int i = 0; i < 3; ++i)
                {
                    v_c[i] = v_c[i] * 0.95;
                    if (std::abs(v_c[i]) < 0.001)
                        v_c[i] = 0.0;
                }
                v_c[3] = 0.0;
                v_c[4] = 0.0;
                v_c[5] = 0.0;
            }

            // 1. Where is the camera currently *supposed* to be?
            vpHomogeneousMatrix bMc_cmd = bMf_cmd * m_fMc;

            // 2. Calculate the small step
            vpPoseVector p_vec(v_c[0] * dt, v_c[1] * dt, v_c[2] * dt, v_c[3] * dt, v_c[4] * dt, v_c[5] * dt);
            vpHomogeneousMatrix delta_cMc;
            delta_cMc.buildFrom(p_vec);

            // 3. Apply the step to get the NEW camera target
            vpHomogeneousMatrix bMc_new = bMc_cmd * delta_cMc;

            // 4. Convert back to Flange target and UPDATE the persistent tracker
            bMf_cmd = bMc_new * m_fMc.inverse();

            // 5. Convert to Flexiv format and stream
            std::array<double, 7> target_pose;
            target_pose[0] = bMf_cmd[0][3];
            target_pose[1] = bMf_cmd[1][3];
            target_pose[2] = bMf_cmd[2][3];

            vpQuaternionVector q(bMf_cmd.getRotationMatrix());
            target_pose[3] = q.w();
            target_pose[4] = q.x();
            target_pose[5] = q.y();
            target_pose[6] = q.z();

            m_robot.StreamCartesianMotionForce(target_pose);
            std::this_thread::sleep_until(next_period);
        }
    }

    flexiv::rdk::Robot m_robot;
    std::unique_ptr<flexiv::rdk::Model> m_model;
    vpCameraParameters m_cam;
    vpHomogeneousMatrix m_fMc;
    SharedVisionData m_shared;
    std::atomic<bool> running;
    std::thread control_thread;
    std::thread monitor_thread;
};

//----------------------------------------------------------
// Main Vision Loop
//----------------------------------------------------------
int main(int argc, char **argv)
{
    if (argc < 2)
        return -1;

    VisualServoApp app(argv[1]);

    // ---------------- RealSense Setup ----------------
    rs2::pipeline pipe;
    rs2::config cfg;

    cfg.enable_stream(RS2_STREAM_COLOR,
                      640,
                      480,
                      RS2_FORMAT_RGB8,
                      30);

    cfg.enable_stream(RS2_STREAM_DEPTH,
                      640,
                      480,
                      RS2_FORMAT_Z16,
                      30);

    pipe.start(cfg);

    rs2::align align_to_color(RS2_STREAM_COLOR);

    // ---------------- ArUco Setup ----------------
    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    cv::Ptr<cv::aruco::DetectorParameters> detectorParams =
        cv::aruco::DetectorParameters::create();

    // ---------------- Start Robot Control ----------------
    app.startThreads();

    std::array<cv::Point2f, 4> empty_corners;

    while (true)
    {
        // Get aligned RealSense frames
        rs2::frameset frames = pipe.wait_for_frames();
        frames = align_to_color.process(frames);

        rs2::video_frame color_frame = frames.get_color_frame();
        rs2::depth_frame depth_frame = frames.get_depth_frame();

        if (!color_frame || !depth_frame)
            continue;

        // Convert RealSense image to OpenCV
        cv::Mat rs_image(
            cv::Size(color_frame.get_width(),
                     color_frame.get_height()),
            CV_8UC3,
            (void *)color_frame.get_data(),
            cv::Mat::AUTO_STEP);

        cv::Mat color;
        cv::cvtColor(rs_image, color, cv::COLOR_RGB2BGR);

        // ----------- Detect ArUco markers -----------
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        std::vector<std::vector<cv::Point2f>> rejectedCandidates;

        cv::aruco::detectMarkers(color,
                                 dictionary,
                                 markerCorners,
                                 markerIds,
                                 detectorParams,
                                 rejectedCandidates);

        if (!markerIds.empty())
        {
            // Use the first detected marker
            size_t idx = 0;

            cv::aruco::drawDetectedMarkers(color,
                                           markerCorners,
                                           markerIds);

            // OpenCV already provides corners in the correct order:
            // 0 = Top-Left
            // 1 = Top-Right
            // 2 = Bottom-Right
            // 3 = Bottom-Left
            std::array<cv::Point2f, 4> corners = {
                markerCorners[idx][0],
                markerCorners[idx][1],
                markerCorners[idx][2],
                markerCorners[idx][3]};

            // Compute marker center
            cv::Point2f center(0.0f, 0.0f);

            for (const auto &pt : markerCorners[idx])
            {
                center += pt;
            }

            center *= 0.25f;

            // Get depth at marker center
            float depth = depth_frame.get_distance(
                static_cast<int>(center.x),
                static_cast<int>(center.y));

            if (depth > 0.01f)
            {

                app.updateVisionData(true, center.x, center.y, depth);

                // Draw marker corners
                for (int i = 0; i < 4; ++i)
                {
                    cv::circle(color,
                               corners[i],
                               5,
                               cv::Scalar(0, 0, 255),
                               -1);

                    cv::putText(color,
                                std::to_string(i),
                                corners[i],
                                cv::FONT_HERSHEY_SIMPLEX,
                                0.5,
                                cv::Scalar(255, 0, 0),
                                1);
                }

                // Draw marker center
                cv::circle(color,
                           center,
                           5,
                           cv::Scalar(255, 0, 0),
                           -1);

                // Draw optical center of D455
                cv::drawMarker(color,
                               cv::Point(326, 245),
                               cv::Scalar(0, 255, 255),
                               cv::MARKER_CROSS,
                               10,
                               1);

                // Display marker ID
                cv::putText(color,
                            "ID: " + std::to_string(markerIds[idx]),
                            cv::Point(static_cast<int>(center.x),
                                      static_cast<int>(center.y) - 15),
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.7,
                            cv::Scalar(0, 255, 0),
                            2);

                // Display depth
                cv::putText(color,
                            "Z: " + std::to_string(depth) + " m",
                            cv::Point(static_cast<int>(center.x),
                                      static_cast<int>(center.y) + 20),
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.6,
                            cv::Scalar(0, 255, 0),
                            2);
            }
            else
            {
                app.updateVisionData(false);
            }
        }
        else
        {
            app.updateVisionData(false);
        }

        cv::imshow("IBVS using ArUco Marker", color);

        if ((cv::waitKey(1) & 0xFF) == 27)
            break;
    }

    pipe.stop();

    return 0;
}