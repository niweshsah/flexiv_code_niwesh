// #include <flexiv/rdk/robot.hpp>
// #include <flexiv/rdk/scheduler.hpp>
// #include <visp3/visual_features/vpFeaturePoint.h>
// #include <visp3/vs/vpServo.h>
// #include <visp3/robot/vpRobotCamera.h>
// #include <mutex>

// // Thread-safe shared data between Vision and Control loops
// struct SharedData {
//     std::mutex mtx;
//     vpFeaturePoint s_current;
//     bool target_found = false;
// };



// class VisualServoApp {
// public:
//     VisualServoApp(flexiv::rdk::Robot& robot) : m_robot(robot) {
        
//         // Initialize ViSP Servo Controller (IBVS)
//         m_task.setServo(vpServo::EYEINHAND_CAMERA);
//         m_task.setInteractionMatrixType(vpServo::CURRENT);
//         m_task.setLambda(0.5); // Control gain
        
//         // Define desired target in image plane
//         m_s_desired.buildFrom(0.0, 0.0, 1.0); // Center of image, Z=1m
//         m_task.addFeature(m_s_current, m_s_desired);
//     }

//     // Runs at 30Hz in background thread
//     void VisionLoop() {
//         while(running) {
//             // 1. Get RealSense frame (using librealsense2)
//             // 2. Detect ArUco marker using ViSP
//             // 3. Update current feature point safely
//             std::lock_guard<std::mutex> lock(m_shared.mtx);
//             m_shared.s_current.buildFrom(x_detected, y_detected, Z_depth);
//             m_shared.target_found = true;
//         }
//     }

//     // Runs at 1000Hz via flexiv::rdk::Scheduler
//     void ControlLoop() {
//         flexiv::rdk::RobotStates state;
//         m_robot.states(state);

//         vpColVector v_c(6); // Camera velocity

//         {
//             std::lock_guard<std::mutex> lock(m_shared.mtx);
//             if(m_shared.target_found) {
//                 // Update feature in ViSP task
//                 m_s_current.set_x(m_shared.s_current.get_x());
//                 m_s_current.set_y(m_shared.s_current.get_y());
//                 m_s_current.set_Z(m_shared.s_current.get_Z());
                
//                 // Compute control law: vc = -lambda * L^+ * e
//                 v_c = m_task.computeControlLaw();
//             } else {
//                 v_c = 0; // Stop if target lost
//             }
//         }

//         // Transform v_c (Camera frame) to v_f (Flange frame)
//         // using your hand-eye calibration matrix eMc
//         vpColVector v_f = m_eMc.inverse() * v_c; 

//         // Map to Flexiv's expected format [vx, vy, vz, wx, wy, wz]
//         std::vector<double> flexiv_vel = {v_f[0], v_f[1], v_f[2], v_f[3], v_f[4], v_f[5]};
        
//         // Send to robot
//         m_robot.SendCartesianVelocity(flexiv_vel);
//     }

// private:
//     flexiv::rdk::Robot& m_robot;

//     vpServo m_task;
//     vpFeaturePoint m_s_current;
//     vpFeaturePoint m_s_desired;
//     vpHomogeneousMatrix m_eMc; // Hand-Eye calibration matrix
//     SharedData m_shared;
//     bool running = true;
// };

int main()
{
    return 0;
}