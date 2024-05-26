#include <gtest/gtest.h>
#include <aruco_detector/aruco_detector.hpp>


class ArucoDetectorTestFixture : public ::testing::Test {
protected:
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    float markerSize;
    cv::Mat cameraMatrix;
    cv::Mat distortionCoeffs;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
    std::unique_ptr<vortex::aruco_detector::ArucoDetector> detector;
    cv::Ptr<cv::aruco::Board> board;

    void SetUp() override {
        // Initialize the variables
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);
        markerSize = 0.167;
        double intrinsic_params[] = {1050.0, 1050.0, 960.0, 540.0}; // fx, fy, cx, cy
        double distortion_params[] = {0.000335365051980971, 0.000583836572965934, 0.0, 0.0, 0.000318839213604595}; // k1, k2, p1, p2, k3
        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 
                intrinsic_params[0], 0, intrinsic_params[2],
                0, intrinsic_params[1], intrinsic_params[3],
                0, 0, 1);

        cv::Mat distortion_coefficients = (cv::Mat_<double>(1, 5) << 
                       distortion_params[0], distortion_params[1], 
                       distortion_params[2], distortion_params[3], 
                       distortion_params[4]);
        cameraMatrix = camera_matrix;
        distortionCoeffs = distortion_coefficients;
        detectorParams = cv::aruco::DetectorParameters::create();
        detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

        // Create an instance of the ArucoDetector
        detector = std::make_unique<vortex::aruco_detector::ArucoDetector>(dictionary, markerSize, cameraMatrix, distortionCoeffs, detectorParams);
        float xDist = 0.462;
        float yDist = 0.862;
        std::vector<int> ids = {28,7,96,19};
        board = detector->createRectangularBoard(markerSize, xDist, yDist, dictionary, ids);
        }
};

TEST_F(ArucoDetectorTestFixture, CreateRectangularBoardTest) {
    // Create a rectangular board
    float xDist = 0.462;
    float yDist = 0.862;
    std::vector<int> ids = {28+1,7+1,96,19+1};
    cv::Ptr<cv::aruco::Board> board = detector->createRectangularBoard(markerSize, xDist, yDist, dictionary, ids);

    // Assert that the board is not null
    ASSERT_TRUE(board != nullptr);

    // Create an image file from the board
    std::string boardImageFile = "/home/jorgen/ros2_ws/src/vortex-sensor-fusion/detectors/aruco_detector/test/board_image.jpg";
    cv::Size imageSize(600, 600); // Set the desired image size
    cv::Mat boardImage;
    cv::aruco::drawPlanarBoard(board, imageSize, boardImage, 10,1);

    // Save the image file
    cv::imwrite(boardImageFile, boardImage);
    // cv::imshow("Board Image", boardImage);
    // cv::waitKey(0);

    // Print the path of the saved image file
    std::cout << "Board image file saved: " << boardImageFile << std::endl;
}

TEST_F(ArucoDetectorTestFixture, DetectMarkersTest) {
    // Load the image
    std::string imageFile = "/home/jorgen/ros2_ws/src/vortex-sensor-fusion/detectors/aruco_detector/test/board_image.jpg";
    cv::Mat image = cv::imread(imageFile, cv::IMREAD_COLOR);

    // Create a copy of the image
    cv::Mat imageCopy = image.clone();

    auto [marker_corners, rejectedCandidates, marker_ids] = detector->detectArucoMarkers(imageCopy);
    // Assert that the ids and corners are not empty
    ASSERT_FALSE(marker_ids.empty());
    ASSERT_FALSE(marker_corners.empty());

    // Draw the detected markers on the copy
    cv::aruco::drawDetectedMarkers(imageCopy, marker_corners, marker_ids);

    // Save the output image
    std::string outputImageFile = "/home/jorgen/ros2_ws/src/vortex-sensor-fusion/detectors/aruco_detector/test/output_image.jpg";
    cv::imwrite(outputImageFile, imageCopy);
}


TEST_F(ArucoDetectorTestFixture, EstimateBoardPoseTest) {
    // Load the image
    std::string imageFile = "/home/jorgen/ros2_ws/src/vortex-sensor-fusion/detectors/aruco_detector/test/board_image.jpg";
    cv::Mat image = cv::imread(imageFile, cv::IMREAD_COLOR);

    // Create a copy of the image
    cv::Mat imageCopy = image.clone();

    auto [marker_corners, rejectedCandidates, marker_ids] = detector->detectArucoMarkers(imageCopy);
    // Assert that the ids and corners are not empty
    ASSERT_FALSE(marker_ids.empty());
    ASSERT_FALSE(marker_corners.empty());

    // Draw the detected markers on the copy
    cv::aruco::drawDetectedMarkers(imageCopy, marker_corners, marker_ids);

    // Save the output image

    // Call the estimateBoardPose function
    auto [boardId, rvec, tvec] = detector->estimateBoardPose(marker_corners, marker_ids, board);
    

    float length = cv::norm(board->objPoints[0][0] - board->objPoints[0][1]); // Visual length of the drawn axis
	        cv::aruco::drawAxis(imageCopy, cameraMatrix, distortionCoeffs, rvec, tvec, length);

    // Print the estimated pose
    std::cout << "Estimated pose:" << std::endl;
    std::cout << "Board ID: " << boardId << std::endl;
    std::cout << "Rotation vector: " << rvec << std::endl;
    std::cout << "Translation vector: " << tvec << std::endl;

    std::string outputImageFile = "/home/jorgen/ros2_ws/src/vortex-sensor-fusion/detectors/aruco_detector/test/drawn_board_image.jpg";
    cv::imwrite(outputImageFile, imageCopy);
    
}


