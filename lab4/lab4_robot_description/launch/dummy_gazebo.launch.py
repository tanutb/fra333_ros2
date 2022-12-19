import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler , TimerAction

from launch_ros.actions import Node
import xacro
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'lab4_robot_description'
    file_subpath = 'description/example_robot.urdf.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'robot'],
                    output='screen')

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["velocity_controllers", "--controller-manager", "/controller_manager"],
    )
    
    package_name = "lab4_robot_description"
    rviz_file_name = 'config_robot.rviz'
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
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        rviz_Node,
        # proximity_detector,
        # tracker,
        # trajectory_generator,
        # marker,
        # Scheduler
        TimerAction(period=3.0,actions=[proximity_detector,
        tracker,
        trajectory_generator,
        marker]),
        TimerAction(period=4.0,actions=[Scheduler])

        
    ])


