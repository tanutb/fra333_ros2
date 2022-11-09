#!usr/bin/python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess,DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
def generate_launch_description():
    # create a place holder for launch description
    launch_description = LaunchDescription()

    ### Example for adding launch argument ###
    # v_max = LaunchConfiguration('v_max')
    # v_max_launch_arg = DeclareLaunchArgument('v_max',default_value='1.0')
    # launch_description.add_action(v_max_launch_arg)
    
    rate = LaunchConfiguration('rate')
    rate_launch_arg = DeclareLaunchArgument('rate',default_value='5.0')
    launch_description.add_action(rate_launch_arg)
    
    ### Example for adding a node ###
    linear = Node(
        package='fra333_lab1_20',
        executable='noise_generator.py',
        namespace= 'linear',
        arguments=[rate],
        # remappings=[
        #     ('/noise','noise'),
        #     ('/set_noise','set_noise')        
        # ]
    )
    launch_description.add_action(linear)
    
    angular = Node(
        package='fra333_lab1_20',
        executable='noise_generator.py',
        namespace= 'angular',
        arguments=[rate],
        # remappings=[
        #     ('/noise','noise'),
        #     ('/set_noise','set_noise')
        # ]
    )
    launch_description.add_action(angular)

    Vmux = Node(
        package='fra333_lab1_20',
        executable='velocity_mux.py',
        arguments=[rate]
    )
    launch_description.add_action(Vmux)

    ### Example for execute a shell command in python script ###
    # vx = 1.0
    # wz = 1.0
    # pub_cmd_vel = ExecuteProcess(
    #     cmd = [[f'ros2 topic pub -r 10 /turtle1/cmd_vel geometry_msgs/msg/Twist "{{linear: {{x: {vx}, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {wz}}}}}"']],
    #     shell=True
    # )
    # launch_description.add_action(pub_cmd_vel)
    
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
    )
    launch_description.add_action(turtlesim_node) 
    
    service_linear_set_noise = ExecuteProcess(
        cmd = [[f'ros2 service call /linear/set_noise lab1_interfaces/srv/SetNoise "{{mean : {{data : 1.0}} , variance : {{data : 0.1}}}}"']],
        shell=True
    )
    launch_description.add_action(service_linear_set_noise)
    service_angular_set_noise = ExecuteProcess(
        cmd = [[f'ros2 service call /angular/set_noise lab1_interfaces/srv/SetNoise "{{mean : {{data : 0.0}} , variance : {{data : 3.0}}}}"']],
        shell=True
    )
    launch_description.add_action(service_angular_set_noise)
    
    return launch_description

    