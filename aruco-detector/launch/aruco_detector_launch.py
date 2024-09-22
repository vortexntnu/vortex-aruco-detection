import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    aruco_detector_node = Node(
        package="aruco_detector",
        executable="aruco_detector_node",
        name="aruco_detector_node",
        parameters=[
            os.path.join(
                get_package_share_directory("aruco_detector"),
                "params",
                "aruco_params.yaml",
            )
        ],
        output="screen",
    )
    return LaunchDescription([aruco_detector_node])
