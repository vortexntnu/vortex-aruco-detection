#include <aruco_detector/aruco_detector_ros.hpp>
#include <sstream>
#include <filesystem>
#include <aruco_detector/aruco_file_logger.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using std::placeholders::_1;

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

    this->declare_parameter<float>("aruco.marker_size", 0.167);
    this->declare_parameter<std::string>("aruco.dictionary", "DICT_5X5_250");

    this->declare_parameter<float>("board.xDist", 0.462);
    this->declare_parameter<float>("board.yDist", 0.862);
    std::vector<int64_t> board_ids = {28, 7, 96, 19};
    this->declare_parameter("board.ids", board_ids);

    this->declare_parameter("models.dynmod_stddev", 0.01);
    this->declare_parameter("models.sensmod_stddev", 0.01);

    setCameraParams();

    checkAndSubscribeToCameraTopics();

    setBoardDetection();

    initializeDetector();


    setVisualization();

    if(detect_board_){
    initializeBoard();
    initializeModels();
    }
    
    setFrame();

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    marker_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/aruco_marker_poses", qos_sensor_data);

    marker_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/aruco_marker_image", qos_sensor_data);

    board_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aruco_board_pose", qos_sensor_data);
    
    toggleKalmanFilterCallback();

    initializeParameterHandler();
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

void ArucoDetectorNode::checkAndSubscribeToCameraTopics() {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    std::string image_topic = this->get_parameter("subs.image_topic").as_string();
    std::string camera_info_topic = this->get_parameter("subs.camera_info_topic").as_string();
    if (image_topic_ != image_topic) {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic, qos_sensor_data, std::bind(&ArucoDetectorNode::imageCallback, this, _1));
        image_topic_ = image_topic;
        RCLCPP_INFO(this->get_logger(), "Subscribed to image topic: %s", image_topic.c_str());
    }
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

    if (msg->distortion_model == "plumb_bob") {
        // Map distortion coefficients
        distortion_coefficients_ = (cv::Mat_<double>(1, 5) << 
                                       msg->d[0], msg->d[1], 0, 0, msg->d[4]);
    } else {
        RCLCPP_WARN(this->get_logger(), "Unsupported distortion model: %s", msg->distortion_model.c_str());
    }
   
    
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

void ArucoDetectorNode::initializeModels() {
    double dynmod_stddev = this->get_parameter("models.dynmod_stddev").as_double();
    double sensmod_stddev = this->get_parameter("models.sensmod_stddev").as_double();

    dynamic_model_ = std::make_shared<DynMod>(dynmod_stddev);
    sensor_model_ = std::make_shared<SensMod>(sensmod_stddev);
}

void ArucoDetectorNode::toggleKalmanFilterCallback() {
    if (detect_board_ && !timer_) { // If detection is enabled and timer is not already running
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ArucoDetectorNode::kalmanFilterCallback, this));
    } else if (!detect_board_ && timer_) { // If detection is disabled, stop and reset the timer
        timer_.reset(); // Stops and destroys the timer
    }
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
        else if (changed_parameter.name.find("models.") == 0) initializeModels();
        else if (changed_parameter.name.find("camera_frame") == 0) setFrame();
        else if (changed_parameter.name.find("subs.") == 0) checkAndSubscribeToCameraTopics();     
    }
    // Directly invoke initializer functions if their related parameters have changed
    if (aruco_changed) {
        initializeDetector();
    }
    if (detect_board_changed) {
        setBoardDetection();
        if(detect_board_){
        initializeBoard();
        initializeModels();
        }
        toggleKalmanFilterCallback();
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

void aruco_detector::ArucoDetectorNode::kalmanFilterCallback()
{
    static rclcpp::Time previous_time = this->now();
    rclcpp::Time current_time = this->now();
    rclcpp::Duration time_since_previous_callback = current_time - previous_time;
    previous_time = current_time;

    auto [status, board_pose_meas, stamp] = board_measurement_.getBoardPoseStamp();
    switch(status) {
    case BoardDetectionStatus::BOARD_NEVER_DETECTED:
        return;
    case BoardDetectionStatus::MEASUREMENT_AVAILABLE:
        std::tie(board_pose_est_, std::ignore, std::ignore) = EKF::step(*dynamic_model_, *sensor_model_, time_since_previous_callback.seconds(),board_pose_est_, board_pose_meas);
        break;
    case BoardDetectionStatus::MEASUREMENT_NOT_AVAILABLE:
        std::tie(board_pose_est_, std::ignore) = EKF::predict(*dynamic_model_, *sensor_model_, time_since_previous_callback.seconds(), board_pose_est_);
        break;
    }
    cv::Vec3d rvec,tvec;
    tvec[0] = board_pose_est_.mean()(0);
    tvec[1] = board_pose_est_.mean()(1);
    tvec[2] = board_pose_est_.mean()(2);
    rvec[0] = board_pose_est_.mean()(3);
    rvec[1] = board_pose_est_.mean()(4);
    rvec[2] = board_pose_est_.mean()(5);
    
    tf2::Quaternion quat = rvec_to_quat(rvec);

    geometry_msgs::msg::PoseStamped pose_msg = cv_pose_to_ros_pose_stamped(tvec, quat, frame_, stamp);
    board_pose_pub_->publish(pose_msg);
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
            Eigen::Vector<double,6> pose(6);
            pose << board_tvec[0], board_tvec[1], board_tvec[2], board_rvec[0], board_rvec[1], board_rvec[2];

            // If the board has never been detected before, the estimated pose is set as the measured pose
            if(std::get<0>(board_measurement_.getBoardPoseStamp()) == BoardDetectionStatus::BOARD_NEVER_DETECTED){
                board_pose_est_ = {pose,Eigen::Matrix<double,6,6>::Identity()};
            }

            rclcpp::Time stamp = msg->header.stamp;
            board_measurement_.setBoardPoseStamp(std::make_tuple(BoardDetectionStatus::MEASUREMENT_AVAILABLE, pose, stamp));

            // If board has been detected, check if rejected markers from the board can be recovered
            std::vector<int> recovered_candidates = aruco_detector_->refineBoardMarkers(input_image_gray, marker_corners, marker_ids, rejected_candidates, board_);

            if(visualize_ && valid > 0){
            // Draw the board axis
            float length = cv::norm(board_->objPoints[0][0] - board_->objPoints[0][1]); // Visual length of the drawn axis
	        cv::aruco::drawAxis(input_image, camera_matrix_, distortion_coefficients_, board_rvec, board_tvec, length);
            }
        }
        else 
        {
            board_measurement_.setBoardStatus(BoardDetectionStatus::MEASUREMENT_NOT_AVAILABLE);
        }
    }
    

    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_size_, camera_matrix_, distortion_coefficients_, rvecs, tvecs);

    geometry_msgs::msg::PoseArray pose_array;

    for (size_t i = 0; i < marker_ids.size(); i++)
    {
        int id = marker_ids[i];
        bool new_id = false;
        static std::string time = ros2TimeToString(msg->header.stamp);
        if (std::find(ids_detected_.begin(), ids_detected_.end(), id) == ids_detected_.end()) {
            ids_detected_.push_back(id);
            new_id = true;
        }
        if (new_id) {
             // Define the directory path
            std::string directory = "detected-aruco-markers/";
            
            // Check if the directory exists, if not, create it
            if (!std::filesystem::exists(directory)) {
                std::filesystem::create_directory(directory);
            }
            
            // Write to the file in the specified directory
            writeIntsToFile(directory + "detected_markers" + time + ".csv", ids_detected_);
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

        // if(detect_board_){
        //     // Draw the board axis
        //     float length = cv::norm(board_->objPoints[0][0] - board_->objPoints[0][1]); // Visual length of the drawn axis
	    //     cv::aruco::drawAxis(input_image, camera_matrix_, distortion_coefficients_, board_rvec, board_tvec, length);
        // }
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
