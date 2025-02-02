#include <aruco_detector/aruco_detector_ros.hpp>
#include <sstream>
#include <filesystem>
#include <aruco_detector/aruco_file_logger.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

namespace vortex {
namespace aruco_detector {

ArucoDetectorNode::ArucoDetectorNode(const rclcpp::NodeOptions & options) : Node("aruco_detector_node", options)
{
    this->declare_parameter<std::string>("camera_frame", "camera_link");

    this->declare_parameter<std::string>("subs.image_topic", "/flir_camera/image_raw");
    this->declare_parameter<std::string>("subs.camera_info_topic", "/flir_camera/camera_info");

    this->declare_parameter<std::vector<double>>("camera.intrinsic",{1050.0, 1050.0, 960.0, 540.0});
    this->declare_parameter<std::vector<double>>("camera.distortion",{0.000335365051980971, 0.000583836572965934, 0.0, 0.0, 0.000318839213604595});

    this->declare_parameter<bool>("detect_board", true);
    this->declare_parameter<bool>("visualize", true);

    this->declare_parameter<float>("aruco.marker_size", 0.150);
    this->declare_parameter<std::string>("aruco.dictionary", "DICT_ARUCO_ORIGINAL");

    this->declare_parameter<float>("board.xDist", 0.430);
    this->declare_parameter<float>("board.yDist", 0.830);
    std::vector<int64_t> board_ids = {28, 7, 96, 19};
    this->declare_parameter("board.ids", board_ids);

    this->declare_parameter("models.dynmod_stddev", 0.01);
    this->declare_parameter("models.sensmod_stddev", 0.01);

    setCameraParams();

    setBoardDetection();

    initializeDetector();


    setVisualization();

    if(detect_board_){
    initializeBoard();
    }
    
    setFrame();

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    marker_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/aruco_marker_poses", qos_sensor_data);

    marker_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/aruco_marker_image", qos_sensor_data);

    board_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aruco_board_pose_camera", qos_sensor_data);

    initializeParameterHandler();

     // Initialize the action server
        action_server_ = rclcpp_action::create_server<vortex_msgs::action::LocateDock>(
            this,
            "search_dock",
            std::bind(&ArucoDetectorNode::handleGoal, this, _1, _2),
            std::bind(&ArucoDetectorNode::handleCancel, this, _1),
            std::bind(&ArucoDetectorNode::handleAccepted, this, _1)
        );
}

rclcpp_action::GoalResponse ArucoDetectorNode::handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const vortex_msgs::action::LocateDock::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to locate dock.");
    (void)uuid;
    (void)goal;
    if (is_executing_action_) {
        RCLCPP_WARN(this->get_logger(), "Already executing an action, rejecting new goal.");
        return rclcpp_action::GoalResponse::REJECT;
    }
    is_executing_action_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ArucoDetectorNode::handleCancel(
    const std::shared_ptr<GoalHandleLocateDock> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal shutting down subscriber.");
    (void)goal_handle;
    image_sub_.reset();
    image_topic_ = "";
    is_executing_action_ = false;
    active_goal_handle_.reset();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ArucoDetectorNode::handleAccepted(const std::shared_ptr<GoalHandleLocateDock> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Goal accepted. Setting up subscriber.");
    subscribeToImageTopic();
    is_executing_action_ = true;
    active_goal_handle_ = goal_handle;
}


void ArucoDetectorNode::setCameraParams(){
    std::vector<double> intrinsic_params = this->get_parameter("camera.intrinsic").as_double_array();
    std::vector<double> distortion_params = this->get_parameter("camera.distortion").as_double_array();

    // Sets default values for camera matrix and distortion coefficients
    // These values will be overwritten by the camera info topic if available
    camera_matrix_ = (cv::Mat_<double>(3, 3) << 
                        intrinsic_params[0], 0, intrinsic_params[2],
                        0, intrinsic_params[1], intrinsic_params[3],
                        0, 0, 1);

    distortion_coefficients_ = (cv::Mat_<double>(1, 5) << 
                                   distortion_params[0], distortion_params[1], 
                                   distortion_params[2], distortion_params[3], 
                                   distortion_params[4]);
}

void ArucoDetectorNode::subscribeToImageTopic() {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    std::string image_topic = this->get_parameter("subs.image_topic").as_string();
    if (image_topic_ != image_topic) {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic, qos_sensor_data, std::bind(&ArucoDetectorNode::imageCallback, this, _1));
        image_topic_ = image_topic;
        RCLCPP_INFO(this->get_logger(), "Subscribed to image topic: %s", image_topic.c_str());
    }
}

void ArucoDetectorNode::subscribeToCameraInfoTopic() {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    std::string camera_info_topic = this->get_parameter("subs.camera_info_topic").as_string();
    if (camera_info_topic_ != camera_info_topic) {
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic, qos_sensor_data, std::bind(&ArucoDetectorNode::cameraInfoCallback, this, _1));
        camera_info_topic_ = camera_info_topic;
        RCLCPP_INFO(this->get_logger(), "Subscribed to camera info topic: %s", camera_info_topic.c_str());
    }
}

void ArucoDetectorNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (msg->k.empty() || msg->d.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Received camera info with empty calibration data.");
    return;
    }

    // Map camera matrix
    camera_matrix_ = (cv::Mat_<double>(3, 3) << 
                        msg->k[0], 0, msg->k[2],
                        0, msg->k[4], msg->k[5],
                        0, 0, 1);

    // Map camera matrix (3x3)
    camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();

    // Map distortion coefficients as a row matrix
    distortion_coefficients_ = cv::Mat(msg->d).reshape(1, 1).clone(); // Reshape to a single row
   
    
     // Correctly using oss for distortion coefficients
    std::ostringstream oss;
    oss << cv::format(distortion_coefficients_, cv::Formatter::FMT_PYTHON);
    std::string formatted_string1 = oss.str();

    // Using a separate ostringstream oss2 for the camera matrix
    std::ostringstream oss2;
    oss2 << cv::format(camera_matrix_, cv::Formatter::FMT_PYTHON);
    std::string formatted_string2 = oss2.str(); // Now correctly using oss2 for the camera matrix

    RCLCPP_INFO(this->get_logger(), "Distortion coefficients: %s", formatted_string1.c_str());
    RCLCPP_INFO(this->get_logger(), "Camera matrix: %s", formatted_string2.c_str());

    RCLCPP_INFO(this->get_logger(), "Camera info received, subscription will be terminated.");
    camera_info_sub_.reset(); // Terminate subscription after receiving camera info
    initializeDetector();
}

void ArucoDetectorNode::initializeDetector() {
    RCLCPP_INFO(this->get_logger(), "Initializing Aruco detector");
    std::string dictionary_type = this->get_parameter("aruco.dictionary").as_string();

    if(dictionary_map.count(dictionary_type)) {
        dictionary_ = cv::aruco::getPredefinedDictionary(dictionary_map[dictionary_type]);
    } else {
        RCLCPP_WARN(this->get_logger(), "Invalid dictionary type received: %s. Using default.", dictionary_type.c_str());
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    }

    detector_params_ = cv::aruco::DetectorParameters::create();

    // Only need accurate aruco pose estimation if board detection is enabled,
    // since operation is time consuming
    if(detect_board_){
        detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    }
    else{
        detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
    }

    marker_size_ = this->get_parameter("aruco.marker_size").as_double();
    aruco_detector_ = std::make_unique<ArucoDetector>(dictionary_, marker_size_, camera_matrix_, distortion_coefficients_, detector_params_);
}

void ArucoDetectorNode::setBoardDetection() {
    detect_board_ = this->get_parameter("detect_board").as_bool();
}

void ArucoDetectorNode::setVisualization() {
    visualize_ = this->get_parameter("visualize").as_bool();
}


void ArucoDetectorNode::initializeBoard() {

    xDist_ = this->get_parameter("board.xDist").as_double();
    yDist_ = this->get_parameter("board.yDist").as_double();
    std::vector<int64_t> param_ids = this->get_parameter("board.ids").as_integer_array();

    // Convert int64_t vector to int vector
    std::vector<int> ids_(param_ids.begin(), param_ids.end());

    board_ = aruco_detector_->createRectangularBoard(marker_size_, xDist_, yDist_, dictionary_, ids_);
}

void ArucoDetectorNode::setFrame() {
    frame_ = this->get_parameter("camera_frame").as_string();
}

void ArucoDetectorNode::initializeParameterHandler() {
    param_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    
    // Register the parameter event callback with the correct signature
    auto parameter_event_callback =
        [this](const rcl_interfaces::msg::ParameterEvent & event) -> void {
            this->onParameterEvent(event);
        };

    // Register the callback with the parameter event handler
    param_cb_handle_ = param_handler_->add_parameter_event_callback(parameter_event_callback);
}

void ArucoDetectorNode::onParameterEvent(const rcl_interfaces::msg::ParameterEvent & event) {
     // Get the fully qualified name of the current node
    auto node_name = this->get_fully_qualified_name();

    // Filter out events not related to this node
    if (event.node != node_name) {
        return; // Early return if the event is not from this node
    }
    RCLCPP_INFO(this->get_logger(), "Received parameter event");

    bool aruco_changed = false;
    bool detect_board_changed = false;
    bool board_changed = false;
    bool camera_changed = false;

    // Determine which groups of parameters have changed
    for (const auto& changed_parameter : event.changed_parameters) {
        if (changed_parameter.name.find("aruco.") == 0) aruco_changed = true;
        else if (changed_parameter.name.find("detect_board") == 0) detect_board_changed = true;
        else if (changed_parameter.name.find("board.") == 0) board_changed = true;
        else if (changed_parameter.name.find("camera.") == 0) camera_changed = true;
        else if (changed_parameter.name.find("camera_frame") == 0) setFrame();
        else if (changed_parameter.name.find("subs.image_topic") == 0) subscribeToImageTopic();
        else if (changed_parameter.name.find("subs.camera_info_topic") == 0) subscribeToCameraInfoTopic();   
    }
    // Directly invoke initializer functions if their related parameters have changed
    if (aruco_changed) {
        initializeDetector();
    }
    if (detect_board_changed) {
        setBoardDetection();
        if(detect_board_){
        initializeBoard();
        }
    }
    if (aruco_changed && detect_board_) {
        initializeBoard();
    }
    if (board_changed) {
        initializeBoard();
    }
    if (camera_changed) {
        setCameraParams();
        initializeDetector();
    }
}


void aruco_detector::ArucoDetectorNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Addr of image message aruco: " << msg.get());
    RCLCPP_INFO_ONCE(this->get_logger(), "Received image message.");
    cv_bridge::CvImagePtr cv_ptr;
        cv::Mat input_image_gray;
        cv::Mat input_image_rgb;
        cv::Mat input_image;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        if (cv_ptr->image.empty()) {
        RCLCPP_WARN(this->get_logger(), "Empty image received, skipping processing.");
        return;
        }

        input_image = cv_ptr->image;

        // cv::cvtColor(input_image, input_image_rgb, cv::COLOR_RGBA2RGB);

        cv::cvtColor(input_image, input_image_rgb, cv::COLOR_BGR2RGB);

	    cv::cvtColor(input_image_rgb, input_image_gray, cv::COLOR_RGB2GRAY);

    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "cv_bridge exception: " << e.what());
        return;
    }


    // markers to vector transforms will be done after refineBoardMarkers in case of recovered candidates
    auto [marker_corners, rejected_candidates, marker_ids] = aruco_detector_->detectArucoMarkers(input_image_gray);


    cv::Vec3d board_rvec, board_tvec;

    if(detect_board_ && marker_ids.size() > 0){
        RCLCPP_INFO_ONCE(this->get_logger(), "Board detection enabled.");
        // Print marker corners
        
        auto [valid, board_rvec, board_tvec] = aruco_detector_->estimateBoardPose(marker_corners, marker_ids, board_);
        
        // valid indicates number of markers used for pose estimation of the board. If valid > 0, a pose has been estimated
        if (valid > 0) {
            auto board_quat = rvec_to_quat(board_rvec);
            geometry_msgs::msg::PoseStamped pose_msg = cv_pose_to_ros_pose_stamped(board_tvec, board_quat, frame_, msg->header.stamp);

            board_pose_pub_->publish(pose_msg);
            // Increment the confirmed counter if the board pose has been confirmed
            if(!confirmed_){
                confirmed_counter_++;
                if(confirmed_counter_ == 5){
                    confirmed_ = true;
                    confirmed_counter_ = 20;
                    auto feedback = std::make_shared<vortex_msgs::action::LocateDock::Feedback>();
                    auto result = std::make_shared<vortex_msgs::action::LocateDock::Result>();
                    feedback->confirmed = true;
                    result->board_pose = pose_msg;
                    active_goal_handle_->publish_feedback(feedback);
                    active_goal_handle_->succeed(result);
                    RCLCPP_INFO(this->get_logger(), "Board pose confirmed.");
                }
            }


            // If board has been detected, check if rejected markers from the board can be recovered
            std::vector<int> recovered_candidates = aruco_detector_->refineBoardMarkers(input_image_gray, marker_corners, marker_ids, rejected_candidates, board_);

            if(visualize_ && valid > 0){
                // Draw the board axis
                float length = cv::norm(board_->objPoints[0][0] - board_->objPoints[0][1]); // Visual length of the drawn axis
                // Only draw board axis if in front of the camera
                if(board_tvec[2] > 0.1){
                    cv::aruco::drawAxis(input_image, camera_matrix_, distortion_coefficients_, board_rvec, board_tvec, length);
                }
            }
        }
        if(valid < 1 && confirmed_){
            confirmed_counter_--;
            if (confirmed_counter_ == 0) {
                confirmed_ = false;
                auto feedback = std::make_shared<vortex_msgs::action::LocateDock::Feedback>();
                feedback->confirmed = false;
                active_goal_handle_->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "Board pose lost.");
            }
        }
    }
    

    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_size_, camera_matrix_, distortion_coefficients_, rvecs, tvecs);

    geometry_msgs::msg::PoseArray pose_array;

    for (size_t i = 0; i < marker_ids.size(); i++)
    {
        int id = marker_ids[i];
        if(id == 0){
            continue;
        }
        id_detection_counter_[id]+=1;

        bool new_id = false;
        static std::string time = ros2TimeToString(msg->header.stamp);

        // If the id is not already in ids_detected_, add it
        if (std::find(ids_detected_once_.begin(), ids_detected_once_.end(), id) == ids_detected_once_.end()) {
            ids_detected_once_.push_back(id);
            new_id = true;
            // Define the directory path
            if(new_id){
            std::string directory = "detected-aruco-markers-stream/";

            // Check if the directory exists, if not, create it
            if (!std::filesystem::exists(directory)) {
                std::filesystem::create_directory(directory);
            }

            // Write to the file in the specified directory
            writeIntsToFile(directory + "detected_markers" + time + ".csv", ids_detected_once_);
            }
        }
        // Check if this id has been detected five times
        if (id_detection_counter_[id] == 5) {
            ids_detected_secure_.push_back(id);
            // Define the directory path
            std::string directory_secure = "detected-aruco-markers-unique/";

            // Check if the directory_secure exists, if not, create it
            if (!std::filesystem::exists(directory_secure)) {
                std::filesystem::create_directory(directory_secure);
            }

            // Write to the file in the specified directory_secure
            writeIntsToFile(directory_secure + "detected_markers" + time + ".csv", ids_detected_secure_);
        }
     
        cv::Vec3d rvec = rvecs[i];
        cv::Vec3d tvec = tvecs[i];
        tf2::Quaternion quat = rvec_to_quat(rvec);

        auto pose_msg = cv_pose_to_ros_pose_stamped(tvec, quat, frame_, msg->header.stamp);
        pose_array.poses.push_back(pose_msg.pose);
    }

    pose_array.header.stamp = msg->header.stamp;
    pose_array.header.frame_id = frame_;
    marker_pose_pub_->publish(pose_array);

    if(visualize_){

        cv::aruco::drawDetectedMarkers(input_image, marker_corners, marker_ids);
        cv::aruco::drawDetectedMarkers(input_image, rejected_candidates, cv::noArray(), cv::Scalar(100, 0, 255));
    
        for (size_t i = 0; i < marker_ids.size(); ++i){
            cv::aruco::drawAxis(input_image, camera_matrix_, distortion_coefficients_, rvecs[i], tvecs[i], 0.1);
        }

        auto message = cv_bridge::CvImage(msg->header, "bgr8", input_image).toImageMsg();


        marker_image_pub_->publish(*message);
    }
}

tf2::Quaternion aruco_detector::ArucoDetectorNode::rvec_to_quat(const cv::Vec3d &rvec) {
    // Convert rotation vector to rotation matrix
    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);
    
    // Convert rotation matrix to quaternion
    tf2::Matrix3x3 rotation_matrix(
        rmat.at<double>(0, 0), rmat.at<double>(0, 1), rmat.at<double>(0, 2),
        rmat.at<double>(1, 0), rmat.at<double>(1, 1), rmat.at<double>(1, 2),
        rmat.at<double>(2, 0), rmat.at<double>(2, 1), rmat.at<double>(2, 2)
    );

    tf2::Quaternion quaternion;
    rotation_matrix.getRotation(quaternion);

    return quaternion;
}

geometry_msgs::msg::PoseStamped aruco_detector::ArucoDetectorNode::cv_pose_to_ros_pose_stamped(const cv::Vec3d &tvec, const tf2::Quaternion &quat, std::string frame_id, rclcpp::Time stamp) {
    // create the PoseStamped message
    geometry_msgs::msg::PoseStamped pose_msg;

    // fill in the header data
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = frame_id;

    // fill in the position data
    pose_msg.pose.position.x = tvec[0];
    pose_msg.pose.position.y = tvec[1];
    pose_msg.pose.position.z = tvec[2];

    // fill in the orientation data
    pose_msg.pose.orientation.x = quat.x();
    pose_msg.pose.orientation.y = quat.y();
    pose_msg.pose.orientation.z = quat.z();
    pose_msg.pose.orientation.w = quat.w();

    // publish the PoseStamped message
    return pose_msg;
}

RCLCPP_COMPONENTS_REGISTER_NODE(vortex::aruco_detector::ArucoDetectorNode)

} // namespace vortex::aruco_detector
} // namespace vortex
