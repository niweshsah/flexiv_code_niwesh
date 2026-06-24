#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/model.hpp>
#include <memory>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>

#include <visp3/vs/vpServo.h>
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
// Configuration Parameters
//----------------------------------------------------------
const double MARKER_SIZE = 0.05;      // 5 cm ArUco marker
const double DESIRED_STANDOFF = 0.30; // 30 cm distance
const double MAX_V = 0.15;            // Max translation velocity (m/s)
const double MAX_W = 0.40;            // Max rotation velocity (rad/s)
const double GAIN_V = 1.5;            // Proportional gain for translation
const double GAIN_W = 1.0;            // Proportional gain for rotation
const double alpha = 0.15;            // Filter factor (0.0 = completely locked/laggy, 1.0 = raw noisy data)

//----------------------------------------------------------
// Thread-Safe Shared Data Structure (PBVS 6-DOF)
//----------------------------------------------------------
struct SharedVisionData
{
    std::mutex mtx;
    bool target_found = false;
    vpHomogeneousMatrix cMo; // Current pose of the object in camera frame
    double safety_damping = 1.0;
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

double clamp(double val, double max_val)
{
    return std::max(-max_val, std::min(val, max_val));
}

//----------------------------------------------------------
// Main PBVS Application Class
//----------------------------------------------------------
class VisualServoApp
{
public:
    VisualServoApp(const std::string &robot_sn) : m_robot(robot_sn), running(true)
    {
        std::cout << "[INFO] [Init] Enabling robot..." << std::endl;
        m_robot.Enable();
        while (!m_robot.operational())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        m_model = std::make_unique<flexiv::rdk::Model>(m_robot);

        // --- RealSense D455 Intrinsics ---
        double px = 386.014465, py = 385.656402;
        double u0 = 325.995117, v0 = 244.844482;
        m_cam.initPersProjWithoutDistortion(px, py, u0, v0);

        // --- Flange to Camera Transform ---
        double tx = -0.125, ty = 0.0, tz = -0.035;
        m_fMc.buildFrom(tx, ty, tz, 0.0, 0.0, -M_PI / 2.0);

        // --- Desired Object Pose relative to Camera (cdMo) ---
        // --- Desired Object Pose relative to Camera (cdMo) ---
        // m_cdMo.buildFrom(0.0, 0.0, DESIRED_STANDOFF, M_PI, 0.0, 0.0);
        m_cdMo.buildFrom(0.0, 0.0, DESIRED_STANDOFF, 0.0, M_PI, 0.0);

        // =========================================================
        // FIX: Modern Flexiv RDK Zeroing Implementation
        // =========================================================
        std::cout << "[INFO] [Init] Switching to NRT_PRIMITIVE_EXECUTION to zero sensors..." << std::endl;
        m_robot.SwitchMode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);

        std::cout << "[INFO] [Init] Executing ZeroFTSensor primitive. Ensure no tool contact!" << std::endl;

        // Pass the bare primitive name alongside an empty map matching the RDK API signature
        m_robot.ExecutePrimitive("ZeroFTSensor", std::map<std::string, flexiv::rdk::FlexivDataTypes>{});

        // Block and query the internal map variant safely to verify completion state
        while (!std::get<int>(m_robot.primitive_states().at("terminated")))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::cout << "[INFO] [Init] F/T sensor zeroed successfully." << std::endl;
        // =========================================================

        std::cout << "[INFO] [Init] RT_CARTESIAN_MOTION_FORCE mode..." << std::endl;
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
        monitor_thread = std::thread(&VisualServoApp::monitorLoop, this);
    }

    void updateVisionData(bool found, const vpHomogeneousMatrix &cMo = vpHomogeneousMatrix())
    {
        std::lock_guard<std::mutex> lock(m_shared.mtx);
        m_shared.target_found = found;
        if (found)
        {
            m_shared.cMo = cMo;
        }
    }

private:
    void monitorLoop()
    {
        while (running)
        {
            auto current_states = m_robot.states();
            m_model->Update(current_states.q, current_states.dtheta);
            auto scores = m_model->configuration_score();

            double trans_score = scores.first;
            double calc_damping = (trans_score < 30.0) ? std::max(0.1, trans_score / 30.0) : 1.0;

            {
                std::lock_guard<std::mutex> lock(m_shared.mtx);
                m_shared.safety_damping = calc_damping;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    void controlLoop()
    {
        std::cout << "[INFO] [Control] 1kHz PBVS Servo Loop Started." << std::endl;
        double dt = 0.001;
        auto next_period = std::chrono::high_resolution_clock::now();
        const auto period = std::chrono::microseconds(1000);

        auto initial_state = m_robot.states();
        cv::Mat T_init = flexivPoseToMatrix(initial_state.flange_pose);
        vpHomogeneousMatrix bMf_cmd;

        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                bMf_cmd[i][j] = T_init.at<double>(i, j);

        vpColVector smoothed_v_c(6, 0.0);

        while (running)
        {
            next_period += period;

            vpColVector raw_v_c(6, 0.0);
            bool valid_target = false;
            vpHomogeneousMatrix current_cMo;
            double damping = 1.0;

            {
                std::lock_guard<std::mutex> lock(m_shared.mtx);
                if (m_shared.target_found)
                {
                    current_cMo = m_shared.cMo;
                    valid_target = true;
                }
                damping = m_shared.safety_damping;
            }

            if (valid_target)
            {
                // 1. Calculate Pose Error
                // The error is the difference between current camera and desired camera frame
                vpHomogeneousMatrix cMcd = current_cMo * m_cdMo.inverse();
                vpPoseVector e(cMcd); // e contains 3 translations and 3 axis-angle rotations

                // 2. Control Law: v = lambda * e
                for (int i = 0; i < 3; ++i)
                {
                    raw_v_c[i] = clamp(damping * GAIN_V * e[i], MAX_V);
                    raw_v_c[i + 3] = clamp(damping * GAIN_W * e[i + 3], MAX_W);
                }

                // 3. Smooth velocities
                double alpha = 0.15;
                for (int i = 0; i < 6; ++i)
                {
                    smoothed_v_c[i] = (alpha * raw_v_c[i]) + ((1.0 - alpha) * smoothed_v_c[i]);
                }
            }
            else
            {
                // Smooth decay to zero if target lost
                for (int i = 0; i < 6; ++i)
                {
                    smoothed_v_c[i] *= 0.95;
                    if (std::abs(smoothed_v_c[i]) < 0.001)
                        smoothed_v_c[i] = 0.0;
                }
            }

            // --- Integrate Spatial Velocity to generate target pose ---
            vpHomogeneousMatrix bMc_cmd = bMf_cmd * m_fMc;

            vpPoseVector p_vec(
                smoothed_v_c[0] * dt, smoothed_v_c[1] * dt, smoothed_v_c[2] * dt,
                smoothed_v_c[3] * dt, smoothed_v_c[4] * dt, smoothed_v_c[5] * dt);

            vpHomogeneousMatrix delta_cMc;
            delta_cMc.buildFrom(p_vec);

            vpHomogeneousMatrix bMc_new = bMc_cmd * delta_cMc;
            bMf_cmd = bMc_new * m_fMc.inverse();

            // --- Stream to Flexiv ---
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
    vpHomogeneousMatrix m_cdMo; // Desired pose
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

    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
    pipe.start(cfg);

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();

    app.startThreads();

    // Setup Camera Matrix for OpenCV PnP
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 386.01, 0, 325.99, 0, 385.65, 244.84, 0, 0, 1);
    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);

    // 3D coordinates of ArUco corners in object frame
    float half_s = static_cast<float>(MARKER_SIZE) / 2.0f;
    std::vector<cv::Point3f> objPoints = {
        {-half_s, half_s, 0.0f},
        {half_s, half_s, 0.0f},
        {half_s, -half_s, 0.0f},
        {-half_s, -half_s, 0.0f}};

    // =========================================================================
    // SMOOTHING FILTER PARAMETERS
    // =========================================================================
    bool first_filtering_done = false;

    // Persistent history variables
    vpTranslationVector filtered_t(0, 0, 0);
    vpQuaternionVector filtered_q(0, 0, 0, 1);
    // =========================================================================

    while (true)
    {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::video_frame color_frame = frames.get_color_frame();
        if (!color_frame)
            continue;

        cv::Mat rs_image(cv::Size(640, 480), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat color;
        cv::cvtColor(rs_image, color, cv::COLOR_RGB2BGR);

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        cv::aruco::detectMarkers(color, dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);

        if (!markerIds.empty())
        {
            cv::aruco::drawDetectedMarkers(color, markerCorners, markerIds);

            // PBVS Modification: Solve PnP for 6D Pose
            cv::Mat rvec, tvec;
            cv::solvePnP(objPoints, markerCorners[0], cameraMatrix, distCoeffs, rvec, tvec);

            // Convert OpenCV rvec/tvec to ViSP vpHomogeneousMatrix
            cv::Mat R;
            cv::Rodrigues(rvec, R);
            vpRotationMatrix vpR;
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    vpR[i][j] = R.at<double>(i, j);

            vpTranslationVector raw_t(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
            vpQuaternionVector raw_q(vpR);

            // =========================================================================
            // APPLY EXPO MOVING AVERAGE (EMA) FILTER
            // =========================================================================
            if (!first_filtering_done)
            {
                // Initialize history with the first measurement
                filtered_t = raw_t;
                filtered_q = raw_q;
                first_filtering_done = true;
            }
            else
            {
                // 1. Smooth Translation Vector element-wise
                filtered_t[0] = (alpha * raw_t[0]) + ((1.0 - alpha) * filtered_t[0]);
                filtered_t[1] = (alpha * raw_t[1]) + ((1.0 - alpha) * filtered_t[1]);
                filtered_t[2] = (alpha * raw_t[2]) + ((1.0 - alpha) * filtered_t[2]);

                // 2. Ensure Quaternions stay in the same hemisphere to prevent sudden 180-deg flips
                if ((raw_q[0] * filtered_q[0] + raw_q[1] * filtered_q[1] + raw_q[2] * filtered_q[2] + raw_q[3] * filtered_q[3]) < 0.0)
                {
                    raw_q = -raw_q;
                }

                // 3. Smooth Rotations via manual NLERP (element-wise blending)
                filtered_q[0] = (alpha * raw_q[0]) + ((1.0 - alpha) * filtered_q[0]);
                filtered_q[1] = (alpha * raw_q[1]) + ((1.0 - alpha) * filtered_q[1]);
                filtered_q[2] = (alpha * raw_q[2]) + ((1.0 - alpha) * filtered_q[2]);
                filtered_q[3] = (alpha * raw_q[3]) + ((1.0 - alpha) * filtered_q[3]);

                // Re-normalize to maintain a valid rotation representation
                filtered_q.normalize();
            }

            // Construct the final jitter-free Homogeneous matrix
            vpRotationMatrix filtered_R(filtered_q);
            vpHomogeneousMatrix cMo_filtered(filtered_t, filtered_R);
            // =========================================================================

            app.updateVisionData(true, cMo_filtered);

            // Draw axis for visual feedback (using the raw PnP parameters or update with filtered ones)
            cv::drawFrameAxes(color, cameraMatrix, distCoeffs, rvec, tvec, 0.05);
            cv::putText(color, "Tracking Filtered 6D Pose", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        }
        else
        {
            // Reset initialization if target disappears for a prolonged period, allowing immediate snap-back next time
            first_filtering_done = false;

            app.updateVisionData(false);
            cv::putText(color, "Searching...", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        }

        cv::imshow("PBVS using ArUco Marker", color);
        if ((cv::waitKey(1) & 0xFF) == 27)
            break;
    }

    pipe.stop();
    return 0;
}