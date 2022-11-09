#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import sys

#sudo apt install ros-foxy-joint-state-publisher
#sudo apt install ros-foxy-joint-state-publisher-gui
def generate_launch_description():

    # RVIZ
    package_name = 'robot20_description'
    rviz_file_name = 'dummy_kinematics.rviz'
    rviz_file_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        rviz_file_name
    )
    rviz_Node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_file_path],
        output='screen')
    robot_desc_path = os.path.join(get_package_share_directory(
                                    'robot20_description'), 
                                    'robot',
                                    'robot20.xarco')

    robot_description = xacro.process_file(robot_desc_path).toxml()


    robot_state_publisher = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='screen',
                                  parameters=[{'robot_description': robot_description}]
    )

    # Launch Description
    launch_description = LaunchDescription()
    launch_description.add_action(rviz_Node)
    launch_description.add_action(robot_state_publisher)

    return launch_description

def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        # quit
        sys.exit()

if __name__ == '__main__':
    main()
    