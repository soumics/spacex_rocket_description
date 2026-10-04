"""RViz-only view of a vehicle with joint sliders (no Gazebo).

    ros2 launch spacex_rocket_description display.launch.py vehicle:=falcon9
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    desc = get_package_share_directory("spacex_rocket_description")
    vehicle = LaunchConfiguration("vehicle").perform(context)
    with open(os.path.join(desc, "urdf", f"{vehicle}.urdf")) as fh:
        urdf = fh.read()
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(desc, "rviz", "vehicle.rviz")]),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("vehicle", default_value="falcon9"),
                              OpaqueFunction(function=setup)])
