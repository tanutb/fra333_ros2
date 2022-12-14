import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler

from launch_ros.actions import Node
import xacro
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    tracker = Node(
    package="lab4_robot_control",
    executable="tracker.py",
    )
    proximity_detector = Node(
    package="lab4_robot_control",
    executable="proximity_detector.py",
    )
    trajectory_generator = Node(
    package="lab4_robot_control",
    executable="trajectory_generator.py",
    )
    marker = Node(
    package="lab4_robot_control",
    executable="marker.py",
    )
    Scheduler = Node(
    package="lab4_robot_control",
    executable="Scheduler.py",
    )

    # Run the node
    return LaunchDescription([
        tracker,
        proximity_detector,
        trajectory_generator,
        marker,
        Scheduler
    ])


