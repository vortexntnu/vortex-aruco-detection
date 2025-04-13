#include <aruco_detector/aruco_file_logger.hpp>

void writeIntsToFile(std::string filename, std::vector<int> ids) {
    std::ofstream outputFile(filename);
    if (outputFile.is_open()) {
        for (int val : ids) {
            outputFile << val << ", ";
        }
    } else
        spdlog::warn(
            "Unable to open file {}, try creating the missing directory",
            filename);
    outputFile.close();
}

std::string ros2TimeToString(const rclcpp::Time& time) {
    auto epoch = std::chrono::seconds{0};  // UNIX epoch start
    auto time_since_epoch = std::chrono::nanoseconds{time.nanoseconds()};
    auto time_point = std::chrono::time_point<std::chrono::system_clock>{
        epoch + time_since_epoch};

    auto time_t_point = std::chrono::system_clock::to_time_t(time_point);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t_point), "%Y-%m-%d %H-%M-%S");

    return ss.str();
}
