#include "pbvs_controller.hpp" // This just copy pastes the header file into this cpp file

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

/*
VERY VERY IMPORTANT:

RUN THIS IN ANOTHER TERMINAL:
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}
*/

/*
 Run as ros2 run flexiv_moveit_codes_niwesh pbvs_controller_node --ros-args --params-file /home/tcs/flexiv_ros2_ws/src/flexiv_moveit_codes_niwesh/config/config.yaml

*/

namespace pbvs_servo
{
    // We update the constructor which is already defined in the header file.
    PbvsController::PbvsController(const rclcpp::NodeOptions &options)
        : Node("pbvs_controller", options), current_state_(ServoState::WAITING_FOR_TARGET), smoothed_twist_(6)
    {
        // Frames matching your Flexiv configuration
        base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
        ee_frame_ = this->declare_parameter<std::string>("ee_frame", "flange");

        target_timeout_sec_ = this->declare_parameter<double>("target_timeout_sec", 0.5);
        continue_to_last_known_target_ = this->declare_parameter<bool>("continue_to_last_known_target", false);

        // Tuning parameters from your yaml
        k_v_ = this->declare_parameter<double>("gain_translation", 1.5);
        k_w_ = this->declare_parameter<double>("gain_rotation", 1.0);
        max_v_ = this->declare_parameter<double>("max_translation_vel", 0.15);
        max_w_ = this->declare_parameter<double>("max_rotation_vel", 0.4);
        desired_standoff_ = this->declare_parameter<double>("desired_standoff", 0.30);

        deadband_v_ = this->declare_parameter<double>("deadband_translation", 0.003);
        deadband_w_ = this->declare_parameter<double>("deadband_rotation", 0.008);

        sign_x_ = this->declare_parameter<int>("sign_x", 1);
        sign_y_ = this->declare_parameter<int>("sign_y", 1);
        sign_z_ = this->declare_parameter<int>("sign_z", 1);

        sign_rx_ = this->declare_parameter<int>("sign_rx", -1);
        sign_ry_ = this->declare_parameter<int>("sign_ry", -1);
        sign_rz_ = this->declare_parameter<int>("sign_rz", 1);

        offset_x_ = this->declare_parameter<double>("offset_x", 0.0);
        offset_y_ = this->declare_parameter<double>("offset_y", 0.0);

        alpha_ = this->declare_parameter<double>("smoothing_alpha", 0.15);

        smoothed_twist_.setZero();

        // -------------------------------------------------------------
        // CONFIGURATION VERIFICATION LOGGING (ADDED)
        // -------------------------------------------------------------
        RCLCPP_INFO(this->get_logger(), "==========================================");
        RCLCPP_INFO(this->get_logger(), "       PBVS CONFIGURATION LOADED         ");
        RCLCPP_INFO(this->get_logger(), "==========================================");
        RCLCPP_INFO(this->get_logger(), "Base Frame:          %s", base_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "EE Frame:            %s", ee_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Timeout Sec:         %.3f", target_timeout_sec_);
        RCLCPP_INFO(this->get_logger(), "Continue on Lost:    %s", continue_to_last_known_target_ ? "TRUE" : "FALSE");
        RCLCPP_INFO(this->get_logger(), "Translation Gain:    %.3f", k_v_);
        RCLCPP_INFO(this->get_logger(), "Rotation Gain:       %.3f", k_w_);
        RCLCPP_INFO(this->get_logger(), "Max Trans Velocity:  %.3f m/s", max_v_);
        RCLCPP_INFO(this->get_logger(), "Max Rot Velocity:    %.3f rad/s", max_w_);
        RCLCPP_INFO(this->get_logger(), "Trans Deadband:      %.4f m", deadband_v_);
        RCLCPP_INFO(this->get_logger(), "Rot Deadband:        %.4f rad", deadband_w_);
        RCLCPP_INFO(this->get_logger(), "Smoothing Alpha:     %.3f", alpha_);
        RCLCPP_INFO(this->get_logger(), "Desired Standoff:    %.3f m", desired_standoff_);

        RCLCPP_INFO(this->get_logger(), "Axis Signs:          [X: %d, Y: %d, Z: %d]", sign_x_, sign_y_, sign_z_);

        RCLCPP_INFO(this->get_logger(), "Rot Axis Signs:      [RX: %d, RY: %d, RZ: %d]", sign_rx_, sign_ry_, sign_rz_);
        
        RCLCPP_INFO(this->get_logger(), "==========================================");

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // It subscribes to the target pose topic and publishes the twist commands to MoveIt Servo
        target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vision/target_pose", 10, std::bind(&PbvsController::targetPoseCallback, this, std::placeholders::_1));
        // targetPoseCallback is called whenever a new target pose message is received

        //  node publishes the twist commands to MoveIt Servo
        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/servo_node/delta_twist_cmds", 10); // 10 is the queue size

        // 100Hz Control Loop
        // This timer calls the controlLoop function at a fixed rate of 100Hz
        // We could have use a while loop with rclcpp::Rate, but using a timer is more in line with ROS2's design and allows for better integration with the executor.

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&PbvsController::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "PBVS Controller Node Started.");
        RCLCPP_INFO(this->get_logger(), "Tracking Target in Tool Frame: [%s]", ee_frame_.c_str());
    }

    // Callback function for receiving target pose messages and transforming them into the end-effector frame. It also updates the servo state based on the received target pose.
    void PbvsController::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {

        std::lock_guard<std::mutex> lock(data_mutex_); // As both targetPoseCallback and controlLoop access shared data, we use a mutex to ensure thread safety when updating the last known target pose and detection time.

        // This lock() works on the current scope only, so no need to manually unlock. It ensures that when we are updating the last_target_pose_ and last_detection_time_, the controlLoop cannot access these variables at the same

        geometry_msgs::msg::PoseStamped latest_msg = *msg;
        // Strip timestamps to ask TF2 for the most recently available transform
        latest_msg.header.stamp.sec = 0;
        latest_msg.header.stamp.nanosec = 0;

        geometry_msgs::msg::PoseStamped target_in_ee;
        try
        {
            target_in_ee = tf_buffer_->transform(latest_msg, ee_frame_, tf2::durationFromSec(0.1));
            last_target_pose_ = target_in_ee;
            last_detection_time_ = this->now();

            // Auto-recovery from stopped state
            if (current_state_ == ServoState::WAITING_FOR_TARGET ||
                current_state_ == ServoState::STOPPED ||
                current_state_ == ServoState::LOST_TARGET_COASTING)
            {
                RCLCPP_INFO(this->get_logger(), "Target acquired! Servoing active.");
            }

            current_state_ = ServoState::SERVOING; // Update the state to SERVOING when a new target is received

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Tracking OK. Target Z-Depth from Tool: %.3f m", target_in_ee.pose.position.z);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF Error in Callback: %s", ex.what());
        }
    }

    // The control loop is responsible for calculating the error between the current end-effector pose and the target pose, applying the PBVS control law, and publishing the resulting twist commands to MoveIt Servo. It also handles state transitions based on target visibility and timeouts.
    void PbvsController::controlLoop()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        auto twist_msg = geometry_msgs::msg::TwistStamped(); // Create a new TwistStamped message

        // Time shift to prevent MoveIt Servo extrapolation errors
        rclcpp::Time safe_time = this->now() - rclcpp::Duration::from_seconds(0.02); // This is a 20ms time shift to prevent MoveIt Servo from extrapolating the command into the future, which can cause errors if the target pose is not updated frequently enough. By using a slightly older timestamp, we ensure that the command is based on a known state of the system.

        twist_msg.header.stamp = safe_time;

        twist_msg.header.frame_id = ee_frame_; // We use ee frame as the reference frame for the twist command, meaning that the linear and angular velocities are expressed relative to the end-effector's coordinate system.

        if (current_state_ == ServoState::SERVOING || current_state_ == ServoState::LOST_TARGET_COASTING)
        {
            double dt = (this->now() - last_detection_time_).seconds();
            if (dt > target_timeout_sec_)
            {
                if (!continue_to_last_known_target_)
                {
                    current_state_ = ServoState::STOPPED;
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Watchdog Timeout! Halting movement.");
                }
                else
                {
                    current_state_ = ServoState::LOST_TARGET_COASTING;
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Target lost, coasting to last known pose...");
                }
            }
        }

        // Handle Stationary States
        if (current_state_ == ServoState::WAITING_FOR_TARGET)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "WAITING FOR FIRST TARGET...");
            twist_pub_->publish(twist_msg); // Publishes zeros
            return;
        }
        if (current_state_ == ServoState::STOPPED)
        {
            // Ramp down smoothly to prevent hardware jerks
            smoothed_twist_ = (1.0 - alpha_) * smoothed_twist_;
            twist_msg.twist.linear.x = smoothed_twist_(0);
            twist_msg.twist.linear.y = smoothed_twist_(1);
            twist_msg.twist.linear.z = smoothed_twist_(2);
            twist_msg.twist.angular.x = smoothed_twist_(3);
            twist_msg.twist.angular.y = smoothed_twist_(4);
            twist_msg.twist.angular.z = smoothed_twist_(5);
            twist_pub_->publish(twist_msg);
            return;
        }

        // -------------------------------------------------------------
        // AXIS INVERSION & STANDOFF TUNING
        // -------------------------------------------------------------

        // SIgns were wrong but now they are correct
        double sign_x = sign_x_;
        double sign_y = sign_y_;
        double sign_z = sign_z_;

        // The distance you want to maintain from the marker (in meters)
        double desired_standoff = desired_standoff_;

        // Apply signs and standoff offset to Linear Error

        // As we are in ee frame, the target's position is relative to the end-effector. So translation error is calculated as the difference between the target's position and the desired standoff distance along the z-axis. The signs are applied to account for the coordinate system orientation.

        Eigen::Vector3d e_v(sign_x * last_target_pose_->pose.position.x + offset_x_,
                            sign_y * last_target_pose_->pose.position.y + offset_y_,
                            sign_z * (last_target_pose_->pose.position.z - desired_standoff));

        // orientation error, we use the quaternion
        Eigen::Quaterniond q_err(last_target_pose_->pose.orientation.w,
                                 last_target_pose_->pose.orientation.x,
                                 last_target_pose_->pose.orientation.y,
                                 last_target_pose_->pose.orientation.z);

        // Convert orientation error to Axis-Angle
        Eigen::AngleAxisd aa_err(q_err); // This converts the quaternion error into an axis-angle representation, which is more intuitive for control purposes. The axis represents the direction of rotation, and the angle represents the magnitude of rotation needed to align the end-effector with the target orientation.

        Eigen::Vector3d raw_e_w = aa_err.axis() * aa_err.angle(); // This gives us a 3D vector representing the rotational error in radians. The direction of the vector indicates the axis of rotation, and its magnitude indicates how much rotation is needed.

        int sign_rx = sign_rx_;
        int sign_ry = sign_ry_;
        int sign_rz = sign_rz_;

        // Apply signs to Rotational Error (SO(3))
        Eigen::Vector3d e_w(sign_rx * raw_e_w(0),
                            sign_ry * raw_e_w(1),
                            sign_rz * raw_e_w(2));

        // Calculate Command Velocities with Deadband and Clamping
        Eigen::VectorXd raw_twist(6);
        for (int i = 0; i < 3; ++i)
        {
            // raw_twist(i) = clampVelocity(k_v_ * applyDeadband(e_v(i), deadband_v_), max_v_);
            // raw_twist(i + 3) = clampVelocity(k_w_ * applyDeadband(e_w(i), deadband_w_), max_w_);

            // Not using deadband for now, as it is being used in ArUco Pose Estimator
            raw_twist(i) = clampVelocity(k_v_ * e_v(i), max_v_);
            raw_twist(i + 3) = clampVelocity(k_w_ * e_w(i), max_w_);
        }

        // Smooth output via Exponential Moving Average (EMA) Filter
        alpha_ = 1.0; // we are not using EMA filter now, as it is being used in MoveIt Servo, 
        smoothed_twist_ = alpha_ * raw_twist + (1.0 - alpha_) * smoothed_twist_;

        // Populate Twist Message
        twist_msg.twist.linear.x = smoothed_twist_(0);
        twist_msg.twist.linear.y = smoothed_twist_(1);
        twist_msg.twist.linear.z = smoothed_twist_(2);
        twist_msg.twist.angular.x = smoothed_twist_(3);
        twist_msg.twist.angular.y = smoothed_twist_(4);
        twist_msg.twist.angular.z = smoothed_twist_(5);

        // Debug Logging at 2Hz
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                             "[PBVS Tool-Relative] Err[x:%.3f, y:%.3f, z:%.3f] | Cmd[x:%.3f, y:%.3f, z:%.3f]",
                             e_v(0), e_v(1), e_v(2),
                             twist_msg.twist.linear.x, twist_msg.twist.linear.y, twist_msg.twist.linear.z);


        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                             "[PBVS Tool-Relative] Err[x:%.3f, y:%.3f, z:%.3f] | Cmd[x:%.3f, y:%.3f, z:%.3f]",
                             e_w(0), e_w(1), e_w(2),
                             twist_msg.twist.angular.x, twist_msg.twist.angular.y, twist_msg.twist.angular.z);

        twist_pub_->publish(twist_msg);
    }

    // Applies a deadband to the error signal. If the absolute value of the error is less than the threshold, it returns zero; otherwise, it subtracts the threshold from the error to reduce sensitivity to small errors.
    double PbvsController::applyDeadband(double error, double threshold)
    {
        if (std::abs(error) < threshold)
            return 0.0;
        return (error > 0) ? (error - threshold) : (error + threshold);
    }

    double PbvsController::clampVelocity(double velocity, double max_vel)
    {
        return std::max(-max_vel, std::min(velocity, max_vel));
    }

} // namespace pbvs_servo

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create an instance of the PbvsController node and spin it to process callbacks and timers. The node will handle target pose messages, compute control commands, and publish them to MoveIt Servo.
    // we use std::make_shared to create a shared pointer to the PbvsController instance, which is a common practice in ROS2 for managing node lifetimes and ensuring proper memory management.
    auto node = std::make_shared<pbvs_servo::PbvsController>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}