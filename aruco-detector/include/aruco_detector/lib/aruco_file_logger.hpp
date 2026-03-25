#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

/**
 * @brief Writes a vector of integers to a file.
 *
 * This function takes a filename and a vector of integers as input and writes
 * the integers to the specified file.
 *
 * @param filename The name of the file to write the integers to.
 * @param ids The vector of integers to be written to the file.
 */
void writeIntsToFile(std::string filename, std::vector<int> ids);

/**
 * @brief Converts nanoseconds since UNIX epoch to a timestamp string.
 *
 * @param nanoseconds since UNIX epoch.
 * @return The string representation on the form "YYYY-MM-DD HH-MM-SS".
 */
std::string nanosecTimeToString(int64_t nanoseconds);

/**
 * @brief Stateful logger that records detected ArUco marker IDs to CSV files.
 *
 * Maintains per-ID detection counters and writes two sets of files:
 *  - stream: updated on every new ID seen
 *  - unique: updated after an ID has been seen secure_write_interval times
 */
class ArucoFileLogger {
   public:
    explicit ArucoFileLogger(int secure_write_interval);

    /**
     * @brief Log a detected marker ID.
     *
     * @param id The detected marker ID.
     * @param time Timestamp string used as part of the output filename.
     */
    void logMarkerId(int id, const std::string& time);

   private:
    int secure_write_interval_;
    std::unordered_map<int, int> id_detection_counter_;
    std::vector<int> ids_detected_once_;
    std::vector<int> ids_detected_secure_;
};
