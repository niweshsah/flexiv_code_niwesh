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
#include <cmath>

//----------------------------------------------------------
// Thread-Safe Shared Data Structure (1 Point)
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
        std::cout << "[INFO] [Init] Enabling robot and waiting for operational state..." << std::endl;
        m_robot.Enable();
        while (!m_robot.operational())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        std::cout << "[INFO] [Init] Configuring ViSP..." << std::endl;

        // m_cam.initPersProjWithoutDistortion(615.0, 615.0, 320.0, 240.0);

        m_cam.initPersProjWithoutDistortion(
            386.119506835938,
            385.761322021484,
            325.9951171875,
            244.844482421875);

        m_task.setServo(vpServo::EYEINHAND_CAMERA);
        m_task.setInteractionMatrixType(vpServo::CURRENT);
        m_task.setLambda(0.4); // Slightly higher gain since rotation is disabled

        double Z_desired = 0.4;
        double x_des, y_des;
        vpPixelMeterConversion::convertPoint(m_cam, 320.0, 240.0, x_des, y_des);

        m_s_desired.buildFrom(x_des, y_des, Z_desired);
        m_task.addFeature(m_s_current, m_s_desired);

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
        std::cout << "[INFO] [Control] 1kHz Servo Loop Started." << std::endl;
        double dt = 0.001;
        uint64_t tick = 0;
        auto next_period = std::chrono::high_resolution_clock::now();
        const auto period = std::chrono::microseconds(1000);

        while (running)
        {
            next_period += period;
            tick++;

            vpColVector v_c(6, 0.0);
            bool valid_target = false;
            {
                std::lock_guard<std::mutex> lock(m_shared.mtx);
                if (m_shared.target_found)
                {
                    m_s_current.buildFrom(m_shared.x_norm, m_shared.y_norm, m_shared.Z_depth);
                    valid_target = true;
                }
            }

            if (valid_target)
            {
                v_c = m_task.computeControlLaw();

                // --- 3-DOF TRANSLATION FIX ---
                // Force pure translation by explicitly zeroing rotation
                v_c[3] = 0.0; // wx
                v_c[4] = 0.0; // wy
                v_c[5] = 0.0; // wz

                v_c[0] = clamp(v_c[0], -0.1, 0.1);
                v_c[1] = clamp(v_c[1], -0.1, 0.1);
                v_c[2] = clamp(v_c[2], -0.1, 0.1);

                if (tick % 1000 == 0)
                {
                    std::cout << "[DEBUG] [Control] Cmd Cam Vel [vx vy vz]: ["
                              << v_c[0] << ", " << v_c[1] << ", " << v_c[2] << "]" << std::endl;
                }
            }

            auto state = m_robot.states();
            cv::Mat T_bf_cv = flexivPoseToMatrix(state.flange_pose);

            vpHomogeneousMatrix bMf_current;
            for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 4; ++j)
                    bMf_current[i][j] = T_bf_cv.at<double>(i, j);

            vpHomogeneousMatrix bMc_current = bMf_current * m_fMc;
            vpPoseVector p_vec(v_c[0] * dt, v_c[1] * dt, v_c[2] * dt, v_c[3] * dt, v_c[4] * dt, v_c[5] * dt);
            vpHomogeneousMatrix delta_cMc;
            delta_cMc.buildFrom(p_vec);

            vpHomogeneousMatrix bMc_cmd = bMc_current * delta_cMc;
            vpHomogeneousMatrix bMf_cmd = bMc_cmd * m_fMc.inverse();

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
        return -1;
    VisualServoApp app(argv[1]);

    rs2::pipeline pipe;
    pipe.start();
    rs2::align align_to_color(RS2_STREAM_COLOR);

    app.startControlThread();

    while (true)
    {
        rs2::frameset frames = pipe.wait_for_frames();
        frames = align_to_color.process(frames);

        rs2::video_frame color_frame = frames.get_color_frame();
        rs2::depth_frame depth_frame = frames.get_depth_frame();
        if (!color_frame || !depth_frame)
            continue;

        cv::Mat rs_image(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat color;
        cv::cvtColor(rs_image, color, cv::COLOR_RGB2BGR);

        cv::Mat hsv, mask1, mask2, mask;
        cv::cvtColor(color, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask1);
        cv::inRange(hsv, cv::Scalar(170, 100, 100), cv::Scalar(180, 255, 255), mask2);
        mask = mask1 | mask2;

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
            double cx = M.m10 / M.m00;
            double cy = M.m01 / M.m00;
            float depth = depth_frame.get_distance(static_cast<int>(cx), static_cast<int>(cy));

            if (depth > 0.01f)
            {
                app.updateVisionData(true, cx, cy, depth);
                cv::circle(color, cv::Point(cx, cy), 5, cv::Scalar(255, 0, 0), -1);
            }
            else
            {
                app.updateVisionData(false, 0, 0, 0);
            }
        }
        else
        {
            app.updateVisionData(false, 0, 0, 0);
        }

        cv::imshow("IBVS 3-DOF", color);
        if ((cv::waitKey(1) & 0xFF) == 27)
            break;
    }
    return 0;
}