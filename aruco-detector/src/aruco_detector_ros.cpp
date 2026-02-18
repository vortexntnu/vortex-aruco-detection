#include <aruco_detector/aruco_detector_ros.hpp>
#include <aruco_detector/aruco_file_logger.hpp>
#include <filesystem>
#include <rclcpp_components/register_node_macro.hpp>
#include <sstream>

using std::placeholders::_1;
using std::placeholders::_2;

namespace {
constexpr int kCovSize = 36;

const std::array<double, kCovSize> IDENTITY_COVARIANCE = [] {
    std::array<double, kCovSize> cov{};
    for (int i = 0; i < 36; i += 7)
        cov[i] = 1.0;
    return cov;
}();
}  // namespace

ArucoDetectorNode::ArucoDetectorNode(const rclcpp::NodeOptions& options)
    : Node("aruco_detector_node", options) {
    detect_board_ = this->declare_parameter<bool>("detect_board");
    visualize_ = this->declare_parameter<bool>("visualize");
    log_markers_ = this->declare_parameter<bool>("log_markers");
    publish_detections_ = this->declare_parameter<bool>("publish_detections");
    publish_landmarks_ = this->declare_parameter<bool>("publish_landmarks");

    this->declare_parameter<float>("aruco.marker_size");
    this->declare_parameter<std::string>("aruco.dictionary");

    this->declare_parameter<float>("board.xDist");
    this->declare_parameter<float>("board.yDist");
    this->declare_parameter<std::vector<int>>("board.ids");

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    std::string image_topic =
        this->declare_parameter<std::string>("subs.image_topic");
    std::string camera_info_topic =
        this->declare_parameter<std::string>("subs.camera_info_topic");

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic, qos_sensor_data,
        std::bind(&ArucoDetectorNode::imageCallback, this, _1));

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, qos_sensor_data,
        std::bind(&ArucoDetectorNode::cameraInfoCallback, this, _1));

    std::string aruco_image_topic =
        this->declare_parameter<std::string>("pubs.aruco_image");

    std::string aruco_poses_topic =
        this->declare_parameter<std::string>("pubs.aruco_poses");

    std::string board_pose_topic =
        this->declare_parameter<std::string>("pubs.board_pose");

    std::string landmarks_topic =
        this->declare_parameter<std::string>("pubs.landmarks");

    marker_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        aruco_image_topic, qos_sensor_data);

    marker_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        aruco_poses_topic, qos_sensor_data);

    board_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        board_pose_topic, qos_sensor_data);

    landmark_pub_ = this->create_publisher<vortex_msgs::msg::LandmarkArray>(
        landmarks_topic, qos_sensor_data);

    std::string logger_service_name =
        this->declare_parameter<std::string>("logger_service_name");

    log_marker_service_ = this->create_service<std_srvs::srv::SetBool>(
        logger_service_name,
        std::bind(&ArucoDetectorNode::toggleLogging, this, _1, _2));
}

void ArucoDetectorNode::toggleLogging(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    log_markers_ = request->data;
    response->success = true;
    response->message =
        log_markers_ ? "Marker logging enabled" : "Marker logging disabled";
    spdlog::info("Marker logging {}", response->message);
}

void ArucoDetectorNode::cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (msg->k.empty()) {
        spdlog::warn(
            "Received camera info with empty camera matrix, skipping.");
        return;
    }
    if (msg->d.empty()) {
        spdlog::warn(
            "Received camera info with empty distortion coefficients, "
            "skipping.");
        return;
    }

    camera_matrix_ =
        cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();

    distortion_coefficients_ =
        cv::Mat(1, msg->d.size(), CV_64F, const_cast<double*>(msg->d.data()))
            .clone();

    spdlog::info("Camera info received");
    camera_info_sub_.reset();
    initializeDetector();
}

void ArucoDetectorNode::initializeDetector() {
    std::string dictionary_type =
        this->get_parameter("aruco.dictionary").as_string();

    if (dictionary_map.count(dictionary_type)) {
        dictionary_ =
            cv::aruco::getPredefinedDictionary(dictionary_map[dictionary_type]);
    } else {
        spdlog::warn("Invalid dictionary type received: {}. Using default.",
                     dictionary_type);
        dictionary_ =
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    }

    detector_params_ = cv::aruco::DetectorParameters::create();

    // Only need accurate aruco pose estimation if board detection is enabled,
    // since operation is time consuming
    if (detect_board_) {
        detector_params_->cornerRefinementMethod =
            cv::aruco::CORNER_REFINE_SUBPIX;
    } else {
        detector_params_->cornerRefinementMethod =
            cv::aruco::CORNER_REFINE_NONE;
    }

    marker_size_ = this->get_parameter("aruco.marker_size").as_double();
    aruco_detector_ = std::make_unique<ArucoDetector>(
        dictionary_, marker_size_, camera_matrix_, distortion_coefficients_,
        detector_params_);

    if (detect_board_) {
        initializeBoard();
    }
}

void ArucoDetectorNode::initializeBoard() {
    xDist_ = this->get_parameter("board.xDist").as_double();
    yDist_ = this->get_parameter("board.yDist").as_double();
    std::vector<int64_t> param_ids =
        this->get_parameter("board.ids").as_integer_array();

    std::vector<int> ids_(param_ids.begin(), param_ids.end());

    board_ = aruco_detector_->createRectangularBoard(marker_size_, xDist_,
                                                     yDist_, dictionary_, ids_);
}

void ArucoDetectorNode::imageCallback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
    if (camera_matrix_.empty() || distortion_coefficients_.empty()) {
        return;
    }
    cv::Mat input_image;
    cv::Mat input_image_gray;
    vortex_msgs::msg::LandmarkArray landmark_array;
    try {
        auto cv_ptr =
            cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        if (cv_ptr->image.empty()) {
            spdlog::warn("Received empty image, skipping processing.");
            return;
        }

        input_image = cv_ptr->image;
        cv::cvtColor(input_image, input_image_gray, cv::COLOR_BGR2GRAY);

    } catch (cv_bridge::Exception& e) {
        spdlog::error("cv_bridge exception: {}", e.what());
        return;
    }

    auto [marker_corners, rejected_candidates, marker_ids] =
        aruco_detector_->detectArucoMarkers(input_image_gray);

    // If no markers are detected, return early
    if (marker_ids.empty()) {
        if (visualize_) {
            auto message = cv_bridge::CvImage(msg->header, "bgr8", input_image)
                               .toImageMsg();
            marker_image_pub_->publish(*message);
        }
        return;
    }

    if (detect_board_) {
        std::vector<int> recovered_candidates =
            aruco_detector_->refineBoardMarkers(input_image_gray,
                                                marker_corners, marker_ids,
                                                rejected_candidates, board_);

        auto [valid, board_rvec, board_tvec] =
            aruco_detector_->estimateBoardPose(marker_corners, marker_ids,
                                               board_);

        if (valid > 0) {
            auto board_quat = rvec_to_quat(board_rvec);
            geometry_msgs::msg::PoseStamped pose_msg =
                cv_pose_to_ros_pose_stamped(board_tvec, board_quat,
                                            msg->header);
            board_pose_pub_->publish(pose_msg);

            vortex_msgs::msg::Landmark board_landmark;
            board_landmark.header = msg->header;
            board_landmark.type_class.type =
                vortex_msgs::msg::LandmarkTypeClass::ARUCO_BOARD;
            board_landmark.type_class.subtype =
                vortex_msgs::msg::LandmarkTypeClass::ARUCO_BOARD_CAMERA;

            geometry_msgs::msg::PoseWithCovariance board_pose_cov;
            board_pose_cov.pose = pose_msg.pose;
            board_pose_cov.covariance = IDENTITY_COVARIANCE;
            board_landmark.pose = board_pose_cov;

            landmark_array.landmarks.push_back(board_landmark);

            if (visualize_) {
                float length =
                    cv::norm(board_->objPoints[0][0] - board_->objPoints[0][1]);
                if (board_tvec[2] > 0.1) {
                    cv::aruco::drawAxis(input_image, camera_matrix_,
                                        distortion_coefficients_, board_rvec,
                                        board_tvec, length);
                }
            }
        }
    }

    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(
        marker_corners, marker_size_, camera_matrix_, distortion_coefficients_,
        rvecs, tvecs);

    geometry_msgs::msg::PoseArray pose_array;

    for (size_t i = 0; i < marker_ids.size(); i++) {
        int id = marker_ids[i];
        if (log_markers_) {
            // id 0 is blacklisted, too many false positives
            if (id == 0) {
                continue;
            }
            static std::string time = ros2TimeToString(msg->header.stamp);

            log_marker_ids(id, time);
        }

        const cv::Vec3d& rvec = rvecs[i];
        const cv::Vec3d& tvec = tvecs[i];
        const tf2::Quaternion quat = rvec_to_quat(rvec);

        auto pose_msg = cv_pose_to_ros_pose_stamped(tvec, quat, msg->header);
        pose_array.poses.push_back(pose_msg.pose);

        vortex_msgs::msg::Landmark landmark;
        landmark.header = msg->header;
        landmark.type_class.type =
            vortex_msgs::msg::LandmarkTypeClass::ARUCO_MARKER;
        landmark.type_class.subtype = static_cast<uint16_t>(id);
        geometry_msgs::msg::PoseWithCovariance pose_cov;
        pose_cov.pose = pose_msg.pose;
        pose_cov.covariance = IDENTITY_COVARIANCE;
        landmark.pose = pose_cov;
        landmark_array.landmarks.push_back(landmark);
    }

    pose_array.header = msg->header;
    landmark_array.header = msg->header;
    if (publish_detections_) {
        marker_pose_pub_->publish(pose_array);
    }
    if (publish_landmarks_ && !landmark_array.landmarks.empty()) {
        landmark_pub_->publish(landmark_array);
    }
    if (visualize_) {
        cv::aruco::drawDetectedMarkers(input_image, marker_corners, marker_ids);
        cv::aruco::drawDetectedMarkers(input_image, rejected_candidates,
                                       cv::noArray(), cv::Scalar(100, 0, 255));

        for (size_t i = 0; i < marker_ids.size(); ++i) {
            cv::aruco::drawAxis(input_image, camera_matrix_,
                                distortion_coefficients_, rvecs[i], tvecs[i],
                                0.1);
        }

        auto message =
            cv_bridge::CvImage(msg->header, "bgr8", input_image).toImageMsg();

        marker_image_pub_->publish(*message);
    }
}

tf2::Quaternion ArucoDetectorNode::rvec_to_quat(const cv::Vec3d& rvec) {
    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);

    tf2::Matrix3x3 rotation_matrix(
        rmat.at<double>(0, 0), rmat.at<double>(0, 1), rmat.at<double>(0, 2),
        rmat.at<double>(1, 0), rmat.at<double>(1, 1), rmat.at<double>(1, 2),
        rmat.at<double>(2, 0), rmat.at<double>(2, 1), rmat.at<double>(2, 2));

    tf2::Quaternion quaternion;
    rotation_matrix.getRotation(quaternion);

    return quaternion;
}

geometry_msgs::msg::PoseStamped ArucoDetectorNode::cv_pose_to_ros_pose_stamped(
    const cv::Vec3d& tvec,
    const tf2::Quaternion& quat,
    const std_msgs::msg::Header& header) {
    geometry_msgs::msg::PoseStamped pose_msg;

    pose_msg.header = header;

    pose_msg.pose.position.x = tvec[0];
    pose_msg.pose.position.y = tvec[1];
    pose_msg.pose.position.z = tvec[2];

    pose_msg.pose.orientation.x = quat.x();
    pose_msg.pose.orientation.y = quat.y();
    pose_msg.pose.orientation.z = quat.z();
    pose_msg.pose.orientation.w = quat.w();

    return pose_msg;
}

void ArucoDetectorNode::log_marker_ids(int id, std::string time) {
    id_detection_counter_[id] += 1;

    bool new_id = false;

    if (std::find(ids_detected_once_.begin(), ids_detected_once_.end(), id) ==
        ids_detected_once_.end()) {
        ids_detected_once_.push_back(id);
        new_id = true;

        if (new_id) {
            std::string directory = "detected-aruco-markers-stream/";

            if (!std::filesystem::exists(directory)) {
                std::filesystem::create_directory(directory);
            }

            writeIntsToFile(directory + "detected_markers" + time + ".csv",
                            ids_detected_once_);
        }
    }

    if (id_detection_counter_[id] == 5) {
        ids_detected_secure_.push_back(id);

        std::string directory_secure = "detected-aruco-markers-unique/";

        if (!std::filesystem::exists(directory_secure)) {
            std::filesystem::create_directory(directory_secure);
        }

        writeIntsToFile(directory_secure + "detected_markers" + time + ".csv",
                        ids_detected_secure_);
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ArucoDetectorNode)
