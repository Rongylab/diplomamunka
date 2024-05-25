# Fungi drone project

## Installation
In order to install the whole system you should follow the following steps.

### Steps

1. Install the ROS2 Hummble on your PC
    https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

2. Install the ArduPilot DDS and its dependencies 
    https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_DDS#installing-build-dependencies

3. Install the Ardupilot Build Environment
    https://ardupilot.org/dev/docs/building-the-code.html#setting-up-the-build-environment
    https://ardupilot.org/dev/docs/building-setup-linux.html

4. Clone the Ardupliot repository to the root of the system
    git clone https://github.com/ArduPilot/ardupilot.git

    If necessary update the submodules with: 
    git submodule update --init --recursive

    or

    git submodule update --recursive --force --init
    git submodule sync --recursive

5. Prepare ROS2 with SITL
    https://ardupilot.org/dev/docs/ros2-sitl.html

6. Install Gazebo Garden
    https://ardupilot.org/dev/docs/ros2-gazebo.html

7. Clone Ardupilot ROS package
    https://github.com/ArduPilot/ardupilot_ros

8. Install DroneKit
    https://dronekit-python.readthedocs.io/en/latest/develop/installation.html

9. Clone this repository

10. Merged the ardupilot_gz and ardupilot_gazebo changes with this repository changes

11. Build the following packages:
    (Before the build source the underlying workspace)
    colcon build --packages-select fungi_drone 
    colcon build --packages-select fungi_drone_v2  
    colcon build --packages-select main_controller 
    colcon build --packages-select fungi_msgs 
    colcon build --packages-select ardupilot_gazebo  
    colcon build --packages-select ardupilot_gz_description 

12. Run the system with the following launch files:
    (Before run the sysetm, source the top of the workspace)
    ros2 launch fungi_drone fungi_sim.launch.py 
    ros2 run fungi_drone_v2 droneController 
    ros2 run main_controller maincontroller 
    ros2 run joy joy_node  
    ros2 run rqt_image_view rqt_image_view 


If there was any issue with the installation process, please check firts the documents/Environment_installation.txt, if it still didn't work, then please don't hesitate to conntact me.
