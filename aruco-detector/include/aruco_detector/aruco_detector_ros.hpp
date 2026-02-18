#ifndef ARUCO_DETECTOR_ROS_HPP
#define ARUCO_DETECTOR_ROS_HPP

#include <rclcpp/rclcpp.hpp>

#include <spdlog/spdlog.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <map>
#include <string>

#include <vortex_msgs/msg/landmark.hpp>
#include <vortex_msgs/msg/landmark_array.hpp>

#include <cv_bridge/cv_bridge.h>

#include "aruco_detector.hpp"

using Vector6d = Eigen::Vector<double, 6>;

static std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME>
    dictionary_map = {{"DICT_4X4_50", cv::aruco::DICT_4X4_50},
                      {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
                      {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
                      {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
                      {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
                      {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
                      {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
                      {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
                      {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
                      {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
                      {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
                      {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
                      {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
                      {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
                      {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
                      {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
                      {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL}};

/**
 * @class ArucoDetectorNode
 * @brief ROS node for ArUco marker detection and pose estimation.
 *
 * This class represents a ROS node that performs ArUco marker detection and
 * pose estimation. Also supports detection of ArUco boards. It subscribes to
 * image and camera info topics, and publishes marker poses, marker images, and
 * board poses. It also provides functionalities for setting camera parameters,
 * initializing the detector, setting visualization options, setting board
 * detection options, initializing the board, initializing the models, and
 * handling parameter events.
 */
class ArucoDetectorNode : public rclcpp::Node {
   public:
    /**
     * @brief Constructs an ArucoDetectorNode object.
     */
    ArucoDetectorNode(const rclcpp::NodeOptions& options);

    /**
     * @brief Destroys the ArucoDetector object.
     */
    ~ArucoDetectorNode() {};

   private:
    /**
     * @brief Subscribes to image topic
     */
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    /**
     * @brief Subscribes to camera info topic to retrieve and set camera
     * parameters
     */
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
        camera_info_sub_;

    /**
     * @brief Publishes marker poses as a PoseArray
     */
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
        marker_pose_pub_;
    /**
     * @brief Publishes the image with the markers visualized if visualization
     * param is set. visualizes the board pose if board detection is set.
     */
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr marker_image_pub_;
    /**
     * @brief Publishes the pose of the board
     */
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        board_pose_pub_;
    /**
     * @brief Publishes detected landmarks as a LandmarkArray
     */
    rclcpp::Publisher<vortex_msgs::msg::LandmarkArray>::SharedPtr landmark_pub_;

    /**
     * @brief Initialize the detector. Sets dictionary from ros param. Also
     * initializes detector parameters.
     */
    void initializeDetector();

    /**
     * @brief Initializes board from ros params
     */
    void initializeBoard();

    /**
     * @brief Callback function for image topic
     *
     * @param msg The image message
     */
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    /**
     * @brief Callback function for camera info topic.
     *  Sets camera matrix and distortion coefficients from camera info message
     * and logs the params.
     *
     * @param msg The camera info message
     */
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    /**
     * @brief Function to convert a rotation vector to a quaternion
     */
    tf2::Quaternion rvec_to_quat(const cv::Vec3d& rvec);

    /**
     * @brief Function to convert a translation vector and quaternion to a
     * PoseStamped message
     */
    geometry_msgs::msg::PoseStamped cv_pose_to_ros_pose_stamped(
        const cv::Vec3d& tvec,
        const tf2::Quaternion& quat,
        const std_msgs::msg::Header& header);

    void log_marker_ids(int id, std::string time);

    void toggleLogging(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr log_marker_service_;

    std::unique_ptr<ArucoDetector> aruco_detector_;

    cv::Mat camera_matrix_, distortion_coefficients_;

    bool detect_board_;
    bool visualize_;
    bool log_markers_;
    bool publish_detections_;
    bool publish_landmarks_;

    float marker_size_;
    float xDist_, yDist_;
    std::vector<int64_t> ids_;
    std::vector<int> ids_detected_once_;
    std::vector<int> ids_detected_secure_;
    std::unordered_map<int, int> id_detection_counter_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    std::string frame_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    cv::Ptr<cv::aruco::Board> board_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;
    std::string image_topic_;
    std::string camera_info_topic_;
    bool camera_info_received_ = false;
};

#endif  // ARUCO_DETECTOR_ROS_HPP
