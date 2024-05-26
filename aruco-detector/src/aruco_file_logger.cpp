#include <aruco_detector/aruco_file_logger.hpp>

void writeIntsToFile(std::string filename, std::vector<int> ids)
{
	std::ofstream outputFile(filename);
	if (outputFile.is_open()) {
		for (int val : ids) {
			outputFile << val << ", ";
		}
	}
	else
        RCLCPP_WARN_STREAM(rclcpp::get_logger("aruco_detector"), "Unable to open file " << filename << ", try creating the missing directory");
	outputFile.close();

}

std::string ros2TimeToString(const rclcpp::Time& time)
{
    auto epoch = std::chrono::seconds{0}; // UNIX epoch start
    auto time_since_epoch = std::chrono::nanoseconds{time.nanoseconds()};
    auto time_point = std::chrono::time_point<std::chrono::system_clock>{epoch + time_since_epoch};

    // Convert time_point to time_t for compatibility with common time functions
    auto time_t_point = std::chrono::system_clock::to_time_t(time_point);

    // Use std::put_time to format the time_t_point as a string
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t_point), "%Y-%m-%d %H-%M-%S");

    // Return the formatted string
    return ss.str();
}

