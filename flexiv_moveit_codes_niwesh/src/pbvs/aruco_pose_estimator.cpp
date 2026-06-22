#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <mutex>

class ArucoPoseEstimator : public rclcpp::Node
{
public:
    ArucoPoseEstimator() : Node("aruco_pose_estimator")
    {
        // 1. Declare Parameters
        marker_size_ = this->declare_parameter<double>("marker_size", 0.0775); // 7.75 cm
        camera_frame_ = this->declare_parameter<std::string>("camera_frame", "camera_color_optical_frame");
        target_id_ = this->declare_parameter<int>("target_marker_id", -1); // -1 means track any detected marker

        // 2. Setup OpenCV ArUco
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        // detector_params_ = cv::aruco::DetectorParameters::create();
        detector_params_ = cv::Ptr<cv::aruco::DetectorParameters>(new cv::aruco::DetectorParameters());
        camera_calibrated_ = false;

        // 3. Publishers
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vision/target_pose", 10);
        debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/vision/debug_image", 10);

        // 4. Subscribers
        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera/color/camera_info", 10,
            std::bind(&ArucoPoseEstimator::cameraInfoCallback, this, std::placeholders::_1));

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10,
            std::bind(&ArucoPoseEstimator::imageCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "ArUco Pose Estimator Node Started.");
    }

private:
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(cam_mutex_);
        if (!camera_calibrated_)
        {
            // Extract Camera Matrix
            camera_matrix_ = (cv::Mat_<double>(3, 3) << msg->k[0], msg->k[1], msg->k[2],
                              msg->k[3], msg->k[4], msg->k[5],
                              msg->k[6], msg->k[7], msg->k[8]);

            // Extract Distortion Coefficients
            dist_coeffs_ = msg->d;
            camera_calibrated_ = true;
            RCLCPP_INFO(this->get_logger(), "Camera Intrinsics Received & Calibrated.");
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(cam_mutex_);
        if (!camera_calibrated_)
            return;

        // Convert ROS Image to OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

        // Detect Markers
        cv::aruco::detectMarkers(cv_ptr->image, dictionary_, markerCorners, markerIds, detector_params_, rejectedCandidates);

        if (!markerIds.empty())
        {
            // Find target marker (or default to the first one seen)
            size_t target_idx = 0;
            if (target_id_ != -1)
            {
                auto it = std::find(markerIds.begin(), markerIds.end(), target_id_);
                if (it != markerIds.end())
                {
                    target_idx = std::distance(markerIds.begin(), it);
                }
                else
                {
                    return; // Target ID not in the image
                }
            }

            // Estimate 6-DoF Pose
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(markerCorners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

            // Draw bounding box and axis on debug image
            cv::aruco::drawDetectedMarkers(cv_ptr->image, markerCorners, markerIds);
            // cv::aruco::drawAxis(cv_ptr->image, camera_matrix_, dist_coeffs_, rvecs[target_idx], tvecs[target_idx], marker_size_ * 0.5f);

            cv::drawFrameAxes(cv_ptr->image, camera_matrix_, dist_coeffs_, rvecs[target_idx], tvecs[target_idx], marker_size_ * 0.5f);

            // Convert OpenCV rvec/tvec to ROS 2 geometry_msgs::PoseStamped
            publishPose(rvecs[target_idx], tvecs[target_idx], msg->header.stamp);
        }

        // Publish debug image for visualization in RViz/Rqt
        sensor_msgs::msg::Image::SharedPtr debug_msg = cv_bridge::CvImage(msg->header, "bgr8", cv_ptr->image).toImageMsg();
        debug_image_pub_->publish(*debug_msg);
    }

    void publishPose(const cv::Vec3d &rvec, const cv::Vec3d &tvec, const rclcpp::Time &stamp)
    {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = camera_frame_;

        // Set Translation (OpenCV uses meters, same as ROS)
        pose_msg.pose.position.x = tvec[0];
        pose_msg.pose.position.y = tvec[1];
        pose_msg.pose.position.z = tvec[2];

        // Convert Rodrigues vector (rvec) to Rotation Matrix
        cv::Mat R;
        cv::Rodrigues(rvec, R);

        // Convert Rotation Matrix to tf2 Quaternion
        tf2::Matrix3x3 tf3d;
        tf3d.setValue(
            R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));

        tf2::Quaternion q;
        tf3d.getRotation(q);

        tf2::Quaternion flip_z;
        flip_z.setRPY(0, M_PI, 0.0); // 180 degrees Roll (around y)
        q = q * flip_z;
        q.normalize();

        // Set Orientation
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        // Publish to the PBVS controller
        pose_pub_->publish(pose_msg);
    }

    // ROS parameters
    double marker_size_;
    std::string camera_frame_;
    int target_id_;

    // OpenCV Data
    cv::Mat camera_matrix_;
    std::vector<double> dist_coeffs_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

    // Thread safety & state
    std::mutex cam_mutex_;
    bool camera_calibrated_;

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoPoseEstimator>());
    rclcpp::shutdown();
    return 0;
}