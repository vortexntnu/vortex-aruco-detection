#pragma once

#include <spdlog/spdlog.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

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
 * @brief Converts a ROS2 Time object to a string representation.
 *
 * @param time The ROS2 Time object to convert.
 * @return The string representation of the ROS2 Time object.
 */
std::string ros2TimeToString(const rclcpp::Time& time);
