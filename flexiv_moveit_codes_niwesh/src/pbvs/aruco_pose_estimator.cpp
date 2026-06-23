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
#include <cmath>

class ArucoPoseEstimator : public rclcpp::Node
{
public:
    ArucoPoseEstimator() : Node("aruco_pose_estimator"), first_measurement_(true), has_published_once_(false)
    {
        // 1. Declare All Parameters (Configurable via YAML)
        marker_size_ = this->declare_parameter<double>("marker_size", 0.0775); // 7.75 cm
        camera_frame_ = this->declare_parameter<std::string>("camera_frame", "camera_color_optical_frame");
        target_id_ = this->declare_parameter<int>("target_marker_id", -1); // -1 means track any detected marker

        // Filter Parameters
        alpha_ = this->declare_parameter<double>("filter_alpha", 0.2);

        // Deadband Parameters
        pos_deadband_ = this->declare_parameter<double>("position_deadband", 0.003); // 3 mm default
        rot_deadband_ = this->declare_parameter<double>("rotation_deadband", 0.035); // ~2 degrees default

        // 2. Setup OpenCV ArUco
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        detector_params_ = cv::Ptr<cv::aruco::DetectorParameters>(new cv::aruco::DetectorParameters());
        camera_calibrated_ = false;

        last_detection_time_ = this->now();

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

        RCLCPP_INFO(this->get_logger(), "ArUco Pose Estimator Node Started with Deadband Filtering.");
    }

private:
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(cam_mutex_);
        if (!camera_calibrated_)
        {
            camera_matrix_ = (cv::Mat_<double>(3, 3) << msg->k[0], msg->k[1], msg->k[2],
                              msg->k[3], msg->k[4], msg->k[5],
                              msg->k[6], msg->k[7], msg->k[8]);

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

        cv::aruco::detectMarkers(cv_ptr->image, dictionary_, markerCorners, markerIds, detector_params_, rejectedCandidates);

        if (!markerIds.empty())
        {
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
                    return;
                }
            }

            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(markerCorners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

            cv::aruco::drawDetectedMarkers(cv_ptr->image, markerCorners, markerIds);
            cv::drawFrameAxes(cv_ptr->image, camera_matrix_, dist_coeffs_, rvecs[target_idx], tvecs[target_idx], marker_size_ * 0.5f);

            processAndPublishPose(rvecs[target_idx], tvecs[target_idx], msg->header.stamp);
        }

        sensor_msgs::msg::Image::SharedPtr debug_msg = cv_bridge::CvImage(msg->header, "bgr8", cv_ptr->image).toImageMsg();
        debug_image_pub_->publish(*debug_msg);
    }

    void processAndPublishPose(const cv::Vec3d &rvec, const cv::Vec3d &tvec, const rclcpp::Time &stamp)
    {
        // 1. Reset filter if marker was lost for > 0.5 seconds
        double dt = (stamp - last_detection_time_).seconds();
        if (dt > 0.5)
        {
            first_measurement_ = true;
            has_published_once_ = false;
        }
        last_detection_time_ = stamp;

        // --- FIX: TRANSFORM OPENCV TO ROS OPTICAL FRAME ---
        // OpenCV: X=Right, Y=Down, Z=Forward
        // If your camera optical frame assumes standard ROS coordinate mapping,
        // you may need to invert Y and X depending on how your frame is defined.
        cv::Vec3d ros_tvec = tvec;
        ros_tvec[0] = -tvec[0]; // X remains Right (or change sign if inverted)
        ros_tvec[1] = -tvec[1]; // Invert Y: OpenCV Down becomes ROS Up/Left convention
        ros_tvec[2] = tvec[2];  // Z remains Forward
        // --------------------------------------------------

        // 2. Update EMA Filter (Translation using transformed vector)
        if (first_measurement_)
        {
            filtered_tvec_ = ros_tvec;
        }
        else
        {
            filtered_tvec_ = (alpha_ * ros_tvec) + ((1.0 - alpha_) * filtered_tvec_);
        }

        // 3. Prepare Current Rotation Quaternion
        cv::Mat R;
        cv::Rodrigues(rvec, R);

        tf2::Matrix3x3 tf3d(
            R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));

        tf2::Quaternion current_q;
        tf3d.getRotation(current_q);

        // --- FIX: ALIGN LOCAL ARUCO AXES ---
        // Post-multiplying by a quaternion rotates the frame LOCALLY.
        // This changes where the Red (X), Green (Y), and Blue (Z) arrows point
        // in RViz without messing up your translation tracking.

        tf2::Quaternion local_alignment;

        // Adjust these angles (in radians) to snap the axes where your PBVS needs them.
        // M_PI/2 is 90 degrees. M_PI is 180 degrees.
        double roll = 0.0;  // Rotate around local X (Red)
        double pitch = M_PI; // Rotate around local Y (Green)
        double yaw = 0.0;   // Rotate around local Z (Blue)

        // Note: If you want X (Red) to point out of the marker face instead of Z,
        // change pitch to: M_PI / 2.0

        local_alignment.setRPY(roll, pitch, yaw);

        // Apply the local rotation
        current_q = current_q * local_alignment;
        current_q.normalize();
        // --------------------------------------------------

        // 4. Update SLERP Filter (Rotation)
        if (first_measurement_)
        {
            filtered_q_ = current_q;
            first_measurement_ = false;
        }
        else
        {
            filtered_q_ = filtered_q_.slerp(current_q, alpha_);
            filtered_q_.normalize();
        }

        // 5. DEADBAND CHECK
        bool should_publish = false;

        if (!has_published_once_)
        {
            should_publish = true;
        }
        else
        {
            double pos_diff = cv::norm(filtered_tvec_ - last_published_tvec_);
            double rot_diff = filtered_q_.angleShortestPath(last_published_q_);

            if (pos_diff > pos_deadband_ || rot_diff > rot_deadband_)
            {
                should_publish = true;
            }
        }

        // 6. Publish if deadband is exceeded
        if (should_publish)
        {
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = stamp;
            pose_msg.header.frame_id = camera_frame_;

            pose_msg.pose.position.x = filtered_tvec_[0];
            pose_msg.pose.position.y = filtered_tvec_[1];
            pose_msg.pose.position.z = filtered_tvec_[2];

            pose_msg.pose.orientation.x = filtered_q_.x();
            pose_msg.pose.orientation.y = filtered_q_.y();
            pose_msg.pose.orientation.z = filtered_q_.z();
            pose_msg.pose.orientation.w = filtered_q_.w();

            pose_pub_->publish(pose_msg);

            last_published_tvec_ = filtered_tvec_;
            last_published_q_ = filtered_q_;
            has_published_once_ = true;
        }
    }

    // ROS parameters
    double marker_size_;
    std::string camera_frame_;
    int target_id_;
    double alpha_;
    double pos_deadband_;
    double rot_deadband_;

    // OpenCV Data
    cv::Mat camera_matrix_;
    std::vector<double> dist_coeffs_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

    // Thread safety & State
    std::mutex cam_mutex_;
    bool camera_calibrated_;

    // Filter & Deadband State
    bool first_measurement_;
    bool has_published_once_;
    cv::Vec3d filtered_tvec_;
    tf2::Quaternion filtered_q_;
    cv::Vec3d last_published_tvec_;
    tf2::Quaternion last_published_q_;
    rclcpp::Time last_detection_time_;

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