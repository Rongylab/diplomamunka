from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_name="example_action"

    # use_sim_time = LaunchConfiguration('use_sim_time')    

    # joy_params = os.path.join(get_package_share_directory(package_name),'config','joystick.yaml')

    server_node_1 = Node(
            package=package_name,
            executable='countdownserver',
            name='server_node_0',
            parameters=[{'ID': 0}],
         )
    
    server_node_2 = Node(
            package=package_name,
            executable='countdownserver',
            name='server_node_1',
            parameters=[{'ID': 1}],
         )

   
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        server_node_1,
        server_node_2,                       
    ])

 