import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

#------------------------------------- AIRDRONE NAVIGATION PKG -----------------------------------------#

    offboard_server = Node(
        package = 'airdrone_navigation',
        name = 'offboard_server',
        executable = 'offboard_control_server',
        parameters = [],
        output = 'screen',
    )
    ld.add_action(offboard_server)

    lading_target_detector = Node(
        package = 'airdrone_navigation',
        name = 'lading_target_detector',
        executable = 'SurfDetector',
        parameters = [],
        output = 'screen',
    )
    ld.add_action(lading_target_detector)

#------------------------------------- CAMERA MANAGER PKG --------------------------------------#

#------------------------------------- FAKE CAMERA PKG -----------------------------------------#
    fake_camera = Node(
        package = 'fake_camera',
        name = 'fake_camera',
        executable = 'fake_camera',
        parameters = [],
        output = 'screen',
    )
    ld.add_action(fake_camera)

#------------------------------------- YOLO ROS PKG -----------------------------------------#

    yolo = Node(
        package = 'yolo_ros',
        name = 'yolo',
        executable = 'yolo',
        parameters = [],
        output = 'screen',
    )
    ld.add_action(yolo)

#------------------------------------- PX4-ROS-COM PKG -----------------------------------------#

    ExecuteProcess(
    cmd=['micrortps_agent', '-t', 'UDP'],
    output='screen'),

#-----------------------------------------------------------------------------------------------#

    return ld