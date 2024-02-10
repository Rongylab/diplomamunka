import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='fungi_drone' #<--- CHANGE ME

     # Setup project paths
    pkg_project_gazebo = get_package_share_directory(package_name)

    x_pose = '0.0'
    y_pose = '0.0'
    z_pose = '0.0'

    # rsp = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory(package_name),'launch','rsp.launch.py'
    #             )]), launch_arguments={'use_sim_time': 'true'}.items()
    # )

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                    )]), launch_arguments={'gz_args': PathJoinSubstitution([
                            pkg_project_gazebo,
                            'worlds',
                            'fungi_hall.sdf'
                        ])}.items(),
                        #'empty.sdf'}.items()
                        # os.path.join(
                        #     get_package_share_directory(package_name), 'worlds', 'basic_test_world.sdf')}.items()
                        #  'empty.sdf'}.items()
             )

    # # Run the create (spawner) node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    # spawn_entity = Node(package='ros_gz_sim', executable='create',
    #                     arguments=[ '-topic', 'robot_description',
    #                                 # '-world', 'new_test_world',
    #                                 '-world', 'basic_test_world',
    #                                 '-entity', 'my_bot',
    #                                 '-x', x_pose,
    #                                 '-y', y_pose,
    #                                 '-z', z_pose,],
    #                     output='screen')
    
    # bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     parameters=[{
    #         'config_file': os.path.join(
    #             get_package_share_directory(package_name), 'config', 'ros_gz_bridge.yaml'),
    #         'qos_overrides./tf_static.publisher.durability': 'transient_local',
    #     }],
    #     output='screen'
    # )

    # rviz2 = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=[ '-d', os.path.join(
    #             get_package_share_directory(package_name), 'config', 'view_bot_cam.rviz'),            
    #     ],
    #     output='screen'
    # )

    # joyStick_teleopTwist = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory(package_name),'launch','joystick.launch.py'
    #             )]), launch_arguments={'use_sim_time': 'true'}.items()
    # )


    # Launch them all!
    return LaunchDescription([
        # rsp,
        gazebo,
        # spawn_entity,
        # bridge,
        # rviz2,
        # joyStick_teleopTwist,
    ])