#include "pbvs_controller.hpp"

namespace pbvs_servo
{

    PbvsController::PbvsController(const rclcpp::NodeOptions &options)
        : Node("pbvs_controller", options), current_state_(ServoState::WAITING_FOR_TARGET), smoothed_twist_(6)
    {
        base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
        ee_frame_ = this->declare_parameter<std::string>("ee_frame", "flange");
        target_timeout_sec_ = this->declare_parameter<double>("target_timeout_sec", 0.5);
        continue_to_last_known_target_ = this->declare_parameter<bool>("continue_to_last_known_target", false);

        k_v_ = this->declare_parameter<double>("gain_translation", 2.5);
        k_w_ = this->declare_parameter<double>("gain_rotation", 1.5);
        max_v_ = this->declare_parameter<double>("max_translation_vel", 0.25);
        max_w_ = this->declare_parameter<double>("max_rotation_vel", 0.6);

        deadband_v_ = this->declare_parameter<double>("deadband_translation", 0.002);
        deadband_w_ = this->declare_parameter<double>("deadband_rotation", 0.005);

        alpha_ = this->declare_parameter<double>("smoothing_alpha", 0.15);

        smoothed_twist_.setZero();

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vision/target_pose", 10, std::bind(&PbvsController::targetPoseCallback, this, std::placeholders::_1));

        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/servo_node/delta_twist_cmds", 10);

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&PbvsController::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "PBVS Controller Node Started successfully.");
        RCLCPP_INFO(this->get_logger(), "Looking for Target in base frame: [%s]", base_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Controlling EE frame: [%s]", ee_frame_.c_str());
    }

    void PbvsController::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        geometry_msgs::msg::PoseStamped latest_msg = *msg;
        latest_msg.header.stamp.sec = 0;
        latest_msg.header.stamp.nanosec = 0;

        geometry_msgs::msg::PoseStamped target_in_base;
        try
        {
            target_in_base = tf_buffer_->transform(latest_msg, base_frame_, tf2::durationFromSec(0.1));
            last_target_pose_ = target_in_base;
            last_detection_time_ = this->now();
            current_state_ = ServoState::SERVOING;
            
            // DEBUG: Prove the callback is getting valid transformed data
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "Target Tracking OK. Target Z-Depth in Base: %.3f", target_in_base.pose.position.z);
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
        twist_msg.header.stamp = this->now();
        twist_msg.header.frame_id = base_frame_;

        if (current_state_ == ServoState::WAITING_FOR_TARGET)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "WAITING FOR TARGET... (Publishing Zero Twist)");
            twist_pub_->publish(twist_msg);
            return;
        }
        else if (current_state_ == ServoState::STOPPED)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "STATE IS STOPPED. (Publishing Zero Twist)");
            twist_pub_->publish(twist_msg);
            return;
        }

        double dt = (this->now() - last_detection_time_).seconds();
        if (dt > target_timeout_sec_)
        {
            if (!continue_to_last_known_target_)
            {
                current_state_ = ServoState::STOPPED;
                RCLCPP_WARN(this->get_logger(), "Watchdog Timeout! Stopping robot movement.");
                twist_pub_->publish(twist_msg);
                return;
            }
            else
            {
                current_state_ = ServoState::LOST_TARGET_COASTING;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Target lost, coasting to last known pose...");
            }
        }

        geometry_msgs::msg::TransformStamped ee_transform;
        try
        {
            ee_transform = tf_buffer_->lookupTransform(base_frame_, ee_frame_, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            // DEBUG: This was failing silently before! 
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "FATAL: Could not lookup EE frame (%s) from Base (%s). Error: %s", 
                                  ee_frame_.c_str(), base_frame_.c_str(), ex.what());
            return;
        }

        Eigen::Vector3d p_c(ee_transform.transform.translation.x, ee_transform.transform.translation.y, ee_transform.transform.translation.z);
        Eigen::Quaterniond q_c(ee_transform.transform.rotation.w, ee_transform.transform.rotation.x, ee_transform.transform.rotation.y, ee_transform.transform.rotation.z);

        Eigen::Vector3d p_d(last_target_pose_->pose.position.x, last_target_pose_->pose.position.y, last_target_pose_->pose.position.z);
        Eigen::Quaterniond q_d(last_target_pose_->pose.orientation.w, last_target_pose_->pose.orientation.x, last_target_pose_->pose.orientation.y, last_target_pose_->pose.orientation.z);

        // Compute Linear Error
        Eigen::Vector3d e_v = p_d - p_c;

        // Compute SO(3) Rotational Error
        Eigen::Quaterniond q_err = q_d * q_c.inverse();
        Eigen::AngleAxisd aa_err(q_err);
        Eigen::Vector3d e_w = aa_err.axis() * aa_err.angle();

        Eigen::VectorXd raw_twist(6);
        for (int i = 0; i < 3; ++i)
        {
            raw_twist(i) = clampVelocity(k_v_ * applyDeadband(e_v(i), deadband_v_), max_v_);
            raw_twist(i + 3) = clampVelocity(k_w_ * applyDeadband(e_w(i), deadband_w_), max_w_);
        }

        smoothed_twist_ = alpha_ * raw_twist + (1.0 - alpha_) * smoothed_twist_;

        twist_msg.twist.linear.x = smoothed_twist_(0);
        twist_msg.twist.linear.y = smoothed_twist_(1);
        twist_msg.twist.linear.z = smoothed_twist_(2);
        twist_msg.twist.angular.x = smoothed_twist_(3);
        twist_msg.twist.angular.y = smoothed_twist_(4);
        twist_msg.twist.angular.z = smoothed_twist_(5);

        // DEBUG: Print Errors and Final Published Command at 2Hz
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
            "[PBVS] Err_Lin[x:%.3f, y:%.3f, z:%.3f] | Cmd_Lin[x:%.3f, y:%.3f, z:%.3f]",
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

// Standalone Main Function Entrypoint
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pbvs_servo::PbvsController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}