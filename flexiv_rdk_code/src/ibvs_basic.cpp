#include <flexiv/rdk/robot.hpp>

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
#include <cmath> // For M_PI

//----------------------------------------------------------
// Thread-Safe Shared Data Structure
//----------------------------------------------------------
struct SharedVisionData
{
    std::mutex mtx;
    bool target_found = false;
    double x_norm = 0.0;
    double y_norm = 0.0;
    double Z_depth = 1.0;
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

        // 1. Enable Robot & Clear Faults (RDK v1.8 Standard Flow)
        std::cout << "[Init] Enabling robot and waiting for operational state..." << std::endl;
        m_robot.Enable();
        while (!m_robot.operational())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        std::cout << "[Init] Configuring ViSP..." << std::endl;

        // 2. Initialize Camera Parameters
        // m_cam.initPersProjWithoutDistortion(615.0, 615.0, 320.0, 240.0);

        m_cam.initPersProjWithoutDistortion(
            386.119506835938,
            385.761322021484,
            325.9951171875,
            244.844482421875);

        // 3. Configure ViSP IBVS Task
        m_task.setServo(vpServo::EYEINHAND_CAMERA);
        m_task.setInteractionMatrixType(vpServo::CURRENT);
        m_task.setLambda(0.3); // Low gain for initial safety

        // 4. Define Desired Feature (Center of screen, 0.4m away)
        double Z_desired = 0.4;
        double x_des, y_des;
        vpPixelMeterConversion::convertPoint(m_cam, 320.0, 240.0, x_des, y_des);

        m_s_desired.buildFrom(x_des, y_des, Z_desired);
        m_task.addFeature(m_s_current, m_s_desired);

        // 5. Set Hand-Eye Calibration Matrix (Flange -> Camera)
        // Values updated to match physical measurements (in meters)
        double tx = -0.125; // -12.5 cm
        double ty = 0.0;    // 0 cm
        double tz = -0.035; // -3.5 cm

        // Rotational correction: -90 degrees around Z-axis maps the frames correctly:
        // Cam_Z = Flange_Z | Cam_Y = Flange_X | Cam_X = -Flange_Y
        m_fMc.buildFrom(tx, ty, tz, 0.0, 0.0, -M_PI / 2.0);

        // 6. Switch to Real-Time Cartesian Control Mode
        std::cout << "[Init] Switching to RT_CARTESIAN_MOTION_FORCE mode..." << std::endl;
        m_robot.SwitchMode(flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE);
    }

    ~VisualServoApp()
    {
        running = false;
        if (control_thread.joinable())
        {
            control_thread.join();
        }
        std::cout << "[Shutdown] Stopping robot..." << std::endl;
        m_robot.Stop();
    }

    void startControlThread()
    {
        control_thread = std::thread(&VisualServoApp::controlLoop, this);
    }

    void updateVisionData(bool found, double px, double py, double depth)
    {
        std::lock_guard<std::mutex> lock(m_shared.mtx);
        m_shared.target_found = found;
        if (found)
        {
            vpPixelMeterConversion::convertPoint(m_cam, px, py, m_shared.x_norm, m_shared.y_norm);
            m_shared.Z_depth = depth;
        }
    }

private:
    void controlLoop()
    {
        std::cout << "[Control] 1kHz Servo Loop Started." << std::endl;
        double dt = 0.001; // 1 ms loop time

        // Setup absolute timing deadlines to ensure rigid real-time 1kHz execution
        auto next_period = std::chrono::high_resolution_clock::now();
        const auto period = std::chrono::microseconds(1000); // 1 millisecond

        while (running)
        {
            next_period += period;

            vpColVector v_c(6, 0.0); // Velocity in camera frame

            // 1. Fetch latest vision data safely
            bool valid_target = false;
            {
                std::lock_guard<std::mutex> lock(m_shared.mtx);
                if (m_shared.target_found)
                {
                    m_s_current.buildFrom(m_shared.x_norm, m_shared.y_norm, m_shared.Z_depth);
                    valid_target = true;
                }
            }

            // 2. Compute ViSP Control Law
            if (valid_target)
            {
                v_c = m_task.computeControlLaw();

                // Absolute Safety Clamping
                v_c[0] = clamp(v_c[0], -0.1, 0.1); // vx (m/s)
                v_c[1] = clamp(v_c[1], -0.1, 0.1); // vy
                v_c[2] = clamp(v_c[2], -0.1, 0.1); // vz
                v_c[3] = clamp(v_c[3], -0.2, 0.2); // wx (rad/s)
                v_c[4] = clamp(v_c[4], -0.2, 0.2); // wy
                v_c[5] = clamp(v_c[5], -0.2, 0.2); // wz
            }

            // 3. Integrate Velocity into Target Pose (RDK v1.8 methodology)
            auto state = m_robot.states();
            cv::Mat T_bf_cv = flexivPoseToMatrix(state.flange_pose);

            vpHomogeneousMatrix bMf_current;
            for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 4; ++j)
                    bMf_current[i][j] = T_bf_cv.at<double>(i, j);

            // Get current camera pose in base frame
            vpHomogeneousMatrix bMc_current = bMf_current * m_fMc;

            // Generate delta-transformation over 1ms step (Exponential Map)
            vpPoseVector p_vec(v_c[0] * dt, v_c[1] * dt, v_c[2] * dt, v_c[3] * dt, v_c[4] * dt, v_c[5] * dt);
            vpHomogeneousMatrix delta_cMc;
            delta_cMc.buildFrom(p_vec);

            // Apply movement step and calculate new flange pose
            vpHomogeneousMatrix bMc_cmd = bMc_current * delta_cMc;
            vpHomogeneousMatrix bMf_cmd = bMc_cmd * m_fMc.inverse();

            // 4. Convert back to Flexiv array format
            std::array<double, 7> target_pose;
            target_pose[0] = bMf_cmd[0][3];
            target_pose[1] = bMf_cmd[1][3];
            target_pose[2] = bMf_cmd[2][3];

            vpQuaternionVector q(bMf_cmd.getRotationMatrix());
            target_pose[3] = q.w();
            target_pose[4] = q.x();
            target_pose[5] = q.y();
            target_pose[6] = q.z();

            // 5. Stream target to the Robot Controller
            m_robot.StreamCartesianMotionForce(target_pose);

            // Sleep precisely until the next absolute 1ms boundary to maintain execution frequency
            std::this_thread::sleep_until(next_period);
        }
    }

    flexiv::rdk::Robot m_robot;
    vpCameraParameters m_cam;
    vpServo m_task;
    vpFeaturePoint m_s_current;
    vpFeaturePoint m_s_desired;
    vpHomogeneousMatrix m_fMc;
    SharedVisionData m_shared;

    std::atomic<bool> running;
    std::thread control_thread;
};

//----------------------------------------------------------
// Main Vision Loop
//----------------------------------------------------------
int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: ./ibvs_red_tracker <robot_sn>\n";
        return -1;
    }

    try
    {
        VisualServoApp app(argv[1]);

        std::cout << "[Vision] Starting RealSense..." << std::endl;
        rs2::pipeline pipe;
        rs2::pipeline_profile profile = pipe.start();
        rs2::align align_to_color(RS2_STREAM_COLOR);

        cv::namedWindow("IBVS Camera View", cv::WINDOW_AUTOSIZE);

        app.startControlThread();

        while (true)
        {
            rs2::frameset frames = pipe.wait_for_frames();
            frames = align_to_color.process(frames);

            rs2::video_frame color_frame = frames.get_color_frame();
            rs2::depth_frame depth_frame = frames.get_depth_frame();
            if (!color_frame || !depth_frame)
                continue;

            cv::Mat rs_image(cv::Size(color_frame.get_width(), color_frame.get_height()),
                             CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat color;
            cv::cvtColor(rs_image, color, cv::COLOR_RGB2BGR);

            cv::Mat hsv, mask1, mask2, mask;
            cv::cvtColor(color, hsv, cv::COLOR_BGR2HSV);

            cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask1);
            cv::inRange(hsv, cv::Scalar(170, 100, 100), cv::Scalar(180, 255, 255), mask2);
            mask = mask1 | mask2;

            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            double max_area = 0.0;
            int largest_idx = -1;
            for (size_t i = 0; i < contours.size(); ++i)
            {
                double area = cv::contourArea(contours[i]);
                if (area > max_area)
                {
                    max_area = area;
                    largest_idx = static_cast<int>(i);
                }
            }

            if (largest_idx != -1 && max_area > 500.0)
            {
                cv::Moments M = cv::moments(contours[largest_idx]);
                if (M.m00 > 0)
                {
                    double cx = M.m10 / M.m00;
                    double cy = M.m01 / M.m00;
                    float depth = depth_frame.get_distance(static_cast<int>(cx), static_cast<int>(cy));

                    if (depth > 0.01f)
                    {
                        app.updateVisionData(true, cx, cy, depth);

                        cv::circle(color, cv::Point(cx, cy), 5, cv::Scalar(255, 0, 0), -1);
                        cv::circle(color, cv::Point(320, 240), 5, cv::Scalar(0, 255, 0), 2);
                    }
                    else
                    {
                        app.updateVisionData(false, 0, 0, 0);
                    }
                }
            }
            else
            {
                app.updateVisionData(false, 0, 0, 0);
            }

            cv::imshow("IBVS Camera View", color);
            if ((cv::waitKey(1) & 0xFF) == 27)
                break;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Fatal Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}