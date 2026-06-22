#ifndef PBVS_CONTROLLER_HPP_
#define PBVS_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mutex>
#include <optional> 

namespace pbvs_servo
{

enum class ServoState {
    WAITING_FOR_TARGET,
    SERVOING,
    LOST_TARGET_COASTING,
    STOPPED
};

class PbvsController : public rclcpp::Node
{
public:
    explicit PbvsController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void controlLoop();

    double applyDeadband(double error, double threshold);
    double clampVelocity(double velocity, double max_vel);

    // ROS Interfaces
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // State & Data
    std::mutex data_mutex_;
    ServoState current_state_;
    std::optional<geometry_msgs::msg::PoseStamped> last_target_pose_;
    rclcpp::Time last_detection_time_;
    
    // Parameters
    std::string base_frame_;
    std::string ee_frame_;
    double target_timeout_sec_;
    bool continue_to_last_known_target_;
    
    double k_v_, k_w_;
    double max_v_, max_w_;
    double deadband_v_, deadband_w_;
    double alpha_;
    
    Eigen::VectorXd smoothed_twist_;
};

} // namespace pbvs_servo

#endif // PBVS_CONTROLLER_HPP_