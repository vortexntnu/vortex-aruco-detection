#include <aruco_detector/aruco_detector.hpp>

ArucoDetector::ArucoDetector(
    cv::Ptr<cv::aruco::Dictionary> dict,
    float marker_size,
    const cv::Mat& camera_matrix,
    const cv::Mat& distortion_coeffs,
    const cv::Ptr<cv::aruco::DetectorParameters> detector_params)
    : dictionary_(dict),
      marker_size_{marker_size},
      camera_matrix_(camera_matrix),
      distortion_coeffs_(distortion_coeffs),
      detector_params_{detector_params} {}

ArucoDetector::~ArucoDetector() {}

cv::Ptr<cv::aruco::Board> ArucoDetector::createRectangularBoard(
    float marker_size,
    float x_dist,
    float y_dist,
    const cv::Ptr<cv::aruco::Dictionary>& dictionary,
    const std::vector<int>& ids) {
    const float marker_half = marker_size / 2;
    const float x_half = x_dist / 2;
    const float y_half = y_dist / 2;

    std::vector<cv::Point3f> marker_centers = {
        {-x_half - marker_half, y_half + marker_half, 0},
        {x_half + marker_half, y_half + marker_half, 0},
        {x_half + marker_half, -y_half - marker_half, 0},
        {-x_half - marker_half, -y_half - marker_half, 0}};

    std::vector<std::vector<cv::Point3f>> marker_points;
    for (const auto& center : marker_centers) {
        std::vector<cv::Point3f> marker = {
            {center.x - marker_half, center.y + marker_half, 0},
            {center.x + marker_half, center.y + marker_half, 0},
            {center.x + marker_half, center.y - marker_half, 0},
            {center.x - marker_half, center.y - marker_half, 0}};
        marker_points.push_back(marker);
    }
    cv::Ptr<cv::aruco::Board> board = new cv::aruco::Board;
    board = cv::aruco::Board::create(marker_points, dictionary, ids);
    return board;

    /*

    top left corner of marker
    |
    X---O               X---O
    |id0|---- xDist ----|id1|
    O---O               O---O
      |                   |
      |                   |
      |                   |
      |                   |
    yDist       O       yDist
      |         |         |
      |      origin       |
      |                   |
      |                   |
    X---O               X---O
    |id3|---- xDist ----|id2|
    O---O               O---O
    |   |
    markerSize

    */
}

std::tuple<std::vector<std::vector<cv::Point2f>>,
           std::vector<std::vector<cv::Point2f>>,
           std::vector<int>>
ArucoDetector::detectArucoMarkers(const cv::Mat& input_image) {
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;

    cv::aruco::detectMarkers(input_image, dictionary_, marker_corners,
                             marker_ids, detector_params_, rejected_candidates);

    return std::make_tuple(marker_corners, rejected_candidates, marker_ids);
}

std::tuple<int, cv::Vec3d, cv::Vec3d> ArucoDetector::estimateBoardPose(
    std::vector<std::vector<cv::Point2f>> marker_corners,
    std::vector<int> marker_ids,
    const cv::Ptr<cv::aruco::Board>& board) {
    cv::Vec3d rvec, tvec;
    int numUsedMarkers = cv::aruco::estimatePoseBoard(
        marker_corners, marker_ids, board, camera_matrix_, distortion_coeffs_,
        rvec, tvec);

    return {numUsedMarkers, rvec, tvec};
}

std::vector<int> ArucoDetector::refineBoardMarkers(
    const cv::Mat& input_image,
    std::vector<std::vector<cv::Point2f>>& corners,
    std::vector<int>& marker_ids,
    std::vector<std::vector<cv::Point2f>>& rejected_candidates,
    cv::Ptr<cv::aruco::Board> board) {
    std::vector<int> recoveredIds;

    cv::aruco::refineDetectedMarkers(input_image, board, corners, marker_ids,
                                     rejected_candidates, camera_matrix_,
                                     distortion_coeffs_, 10, 3, true,
                                     recoveredIds, detector_params_);

    return recoveredIds;
}
