#include <aruco_detector/lib/aruco_file_logger.hpp>

#include <spdlog/spdlog.h>
#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

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

std::string nanosecTimeToString(int64_t nanoseconds) {
    auto time_since_epoch = std::chrono::nanoseconds{nanoseconds};
    auto time_point =
        std::chrono::time_point<std::chrono::system_clock>{time_since_epoch};
    auto time_t_point = std::chrono::system_clock::to_time_t(time_point);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t_point), "%Y-%m-%d %H-%M-%S");

    return ss.str();
}

ArucoFileLogger::ArucoFileLogger(int secure_write_interval, logMode mode)
    : secure_write_interval_(secure_write_interval), mode_(mode) {}

void ArucoFileLogger::logMarkerId(int id, const std::string& time) {
    id_detection_counter_[id] += 1;

    if (mode_ == logMode::STREAM) {
        if (std::find(ids_detected_once_.begin(), ids_detected_once_.end(),
                      id) == ids_detected_once_.end()) {
            ids_detected_once_.push_back(id);

            std::string directory = "detected-aruco-markers-stream/";
            if (!std::filesystem::exists(directory)) {
                std::filesystem::create_directory(directory);
            }
            writeIntsToFile(directory + "detected_markers" + time + ".csv",
                            ids_detected_once_);
        }
    } else {
        if (id_detection_counter_[id] == secure_write_interval_) {
            ids_detected_secure_.push_back(id);

            std::string directory = "detected-aruco-markers-unique/";
            if (!std::filesystem::exists(directory)) {
                std::filesystem::create_directory(directory);
            }
            writeIntsToFile(directory + "detected_markers" + time + ".csv",
                            ids_detected_secure_);
        }
    }
}
