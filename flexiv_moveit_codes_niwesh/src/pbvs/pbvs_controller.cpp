#include "pbvs_controller.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

'''

VERY VERY IMPORTANT

RUN THIS IN ANOTHER TERMINAL: ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}


'''

namespace pbvs_servo
{

    PbvsController::PbvsController(const rclcpp::NodeOptions &options)
        : Node("pbvs_controller", options), current_state_(ServoState::WAITING_FOR_TARGET), smoothed_twist_(6)
    {
        // Frames matching your Flexiv configuration
        base_frame_ = this->declare_parameter<std::string>("base_frame", "Rizon4s-063387_base_link");
        ee_frame_ = this->declare_parameter<std::string>("ee_frame", "Rizon4s-063387_flange");

        target_timeout_sec_ = this->declare_parameter<double>("target_timeout_sec", 0.5);
        continue_to_last_known_target_ = this->declare_parameter<bool>("continue_to_last_known_target", false);

        // Tuning parameters from your yaml
        k_v_ = this->declare_parameter<double>("gain_translation", 1.5);
        k_w_ = this->declare_parameter<double>("gain_rotation", 1.0);
        max_v_ = this->declare_parameter<double>("max_translation_vel", 0.15);
        max_w_ = this->declare_parameter<double>("max_rotation_vel", 0.4);

        deadband_v_ = this->declare_parameter<double>("deadband_translation", 0.003);
        deadband_w_ = this->declare_parameter<double>("deadband_rotation", 0.008);

        alpha_ = this->declare_parameter<double>("smoothing_alpha", 0.15);

        smoothed_twist_.setZero();

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vision/target_pose", 10, std::bind(&PbvsController::targetPoseCallback, this, std::placeholders::_1));

        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/servo_node/delta_twist_cmds", 10);

        // 100Hz Control Loop
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&PbvsController::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "PBVS Controller Node Started.");
        RCLCPP_INFO(this->get_logger(), "Tracking Target in Tool Frame: [%s]", ee_frame_.c_str());
    }

    void PbvsController::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        geometry_msgs::msg::PoseStamped latest_msg = *msg;
        // Strip timestamps to ask TF2 for the most recently available transform
        latest_msg.header.stamp.sec = 0;
        latest_msg.header.stamp.nanosec = 0;

        geometry_msgs::msg::PoseStamped target_in_ee;
        try
        {
            // METHOD 1 FIX: Transform the target directly into the End-Effector Frame
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
            current_state_ = ServoState::SERVOING;

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Tracking OK. Target Z-Depth from Tool: %.3f m", target_in_ee.pose.position.z);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF Error in Callback: %s", ex.what());
        }
    }

    void PbvsController::controlLoop()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        auto twist_msg = geometry_msgs::msg::TwistStamped();

        // Time shift to prevent MoveIt Servo extrapolation errors
        rclcpp::Time safe_time = this->now() - rclcpp::Duration::from_seconds(0.02);
        twist_msg.header.stamp = safe_time;

        // METHOD 1 FIX: Explicitly tell MoveIt Servo these are Tool-Relative commands
        twist_msg.header.frame_id = ee_frame_;

        // Watchdog evaluation
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

        // METHOD 1 MATH: Because last_target_pose_ is already in the EE frame,
        // the camera/target position *is* the mathematical error.
        // We do not need to subtract the current pose, because the EE is at (0,0,0) relative to itself.

        // -------------------------------------------------------------
        // AXIS INVERSION & STANDOFF TUNING
        // -------------------------------------------------------------
        double sign_x = -1.0;
        double sign_y = -1.0;
        double sign_z = 1.0;

        // The distance you want to maintain from the marker (in meters)
        double desired_standoff = 0.30;

        // Apply signs and standoff offset to Linear Error
        Eigen::Vector3d e_v(sign_x * last_target_pose_->pose.position.x,
                            sign_y * last_target_pose_->pose.position.y,
                            sign_z * (last_target_pose_->pose.position.z - desired_standoff));

        Eigen::Quaterniond q_err(last_target_pose_->pose.orientation.w,
                                 last_target_pose_->pose.orientation.x,
                                 last_target_pose_->pose.orientation.y,
                                 last_target_pose_->pose.orientation.z);

        // Convert orientation error to Axis-Angle
        Eigen::AngleAxisd aa_err(q_err);
        Eigen::Vector3d raw_e_w = aa_err.axis() * aa_err.angle();

        // Apply signs to Rotational Error (SO(3))
        Eigen::Vector3d e_w(sign_x * raw_e_w(0),
                            sign_y * raw_e_w(1),
                            sign_z * raw_e_w(2));

        // Calculate Command Velocities with Deadband and Clamping
        Eigen::VectorXd raw_twist(6);
        for (int i = 0; i < 3; ++i)
        {
            raw_twist(i) = clampVelocity(k_v_ * applyDeadband(e_v(i), deadband_v_), max_v_);
            raw_twist(i + 3) = clampVelocity(k_w_ * applyDeadband(e_w(i), deadband_w_), max_w_);
        }

        // Smooth output via Exponential Moving Average (EMA) Filter
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

        twist_pub_->publish(twist_msg);
    }

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
    auto node = std::make_shared<pbvs_servo::PbvsController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
