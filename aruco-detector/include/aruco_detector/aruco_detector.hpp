#ifndef ARUCO_DETECTOR_HPP
#define ARUCO_DETECTOR_HPP

#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

/**
 * @brief Class for detecting and estimating poses of ArUco markers. Also
 * supports detection of ArUco boards.
 */
class ArucoDetector {
   public:
    /**
     * @brief Constructs an ArucoDetector object.
     *
     * @param dict The dictionary of ArUco markers.
     * @param marker_size The size of the markers in meters.
     * @param camera_matrix The camera matrix.
     * @param distortion_coeffs The distortion coefficients.
     * @param detector_params The parameters for the marker detector.
     */
    ArucoDetector(cv::Ptr<cv::aruco::Dictionary> dict,
                  float marker_size,
                  const cv::Mat& camera_matrix,
                  const cv::Mat& distortion_coeffs,
                  const cv::Ptr<cv::aruco::DetectorParameters> detector_params);

    /**
     * @brief Destroys the ArucoDetector object.
     */
    ~ArucoDetector();

    /**
     * @brief Detects ArUco markers in the input image.
     *
     * @param input_image The input image.
     * @return A tuple containing the detected marker corners, rejected
     * candidates, and marker IDs.
     */
    std::tuple<std::vector<std::vector<cv::Point2f>>,
               std::vector<std::vector<cv::Point2f>>,
               std::vector<int>>
    detectArucoMarkers(const cv::Mat& input_image);

    /**
     * @brief Estimates the pose of a board composed of ArUco markers.
     *
     * @param marker_corners The corners of the detected markers.
     * @param marker_ids The IDs of the detected markers.
     * @param board The ArUco board.
     * @return A tuple containing the number of markers used for pose
     * estimation, the translation vector, and the rotation vector. If the
     * number is greater than 0, a board post has been estimated and the
     * translation and rotation vectors contain the estimated pose.
     *
     */
    std::tuple<int, cv::Vec3d, cv::Vec3d> estimateBoardPose(
        std::vector<std::vector<cv::Point2f>> marker_corners,
        std::vector<int> marker_ids,
        const cv::Ptr<cv::aruco::Board>& board);

    /**
     * @brief Creates ArUco board.
     *
     * @param markerSize The size of each marker in meters.
     * @param xDist The distance between markers along the X-axis.
     * @param yDist The distance between markers along the Y-axis.
     * @param dictionary The dictionary of ArUco markers.
     * @param ids The IDs of the markers to be used in the board.
     * @return The created ArUco board.
     */
    cv::Ptr<cv::aruco::Board> createRectangularBoard(
        float markerSize,
        float xDist,
        float yDist,
        const cv::Ptr<cv::aruco::Dictionary>& dictionary,
        const std::vector<int>& ids);

    /**
     * @brief Tries to recover rejected markers from the rejected candidates.
     *  If one detected marker is from the board, we can use it to estimate
     * rejected candidates from within the board.
     *
     * @param input_image The input image.
     * @param corners The detected marker corners.
     * @param ids The detected marker IDs.
     * @param rejected_candidates The rejected marker candidates.
     * @param board The ArUco board.
     * @return The refined marker IDs.
     */
    std::vector<int> refineBoardMarkers(
        const cv::Mat& input_image,
        std::vector<std::vector<cv::Point2f>>& corners,
        std::vector<int>& ids,
        std::vector<std::vector<cv::Point2f>>& rejected_candidates,
        cv::Ptr<cv::aruco::Board> board);

   private:
    cv::Ptr<cv::aruco::Dictionary>
        dictionary_;    /**< The dictionary of ArUco markers. */
    float marker_size_; /**< The size of the markers in meters. */
    float xDist_;       /**< The distance between markers in the board along the
                           X-axis. */
    float yDist_;       /**< The distance between markers in the board along the
                           Y-axis. */
    cv::Mat camera_matrix_;     /**< The camera matrix. */
    cv::Mat distortion_coeffs_; /**< The distortion coefficients. */
    cv::Ptr<cv::aruco::DetectorParameters>
        detector_params_; /**< The parameters for the marker detector. */
};

#endif  // ARUCO_DETECTOR_HPP
