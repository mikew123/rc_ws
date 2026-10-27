# .bashrc has an alias "rc" which builds launches this project
# as well as the delphi_descriptins urdf launch file
# The extra delayed urdf launch fixes the oak frame issues when nav2 is launched
#
# alias rc=' \
#  cd ~/rc_ws ; colcon build ; source install/setup.bash ; \
#  parallel --lb ::: \
#  "ros2 launch robocolumbus_stuff bringup_launch.py" \
#  "sleep 5 ; ros2 launch depthai_descriptions urdf_launch.py" ; \
#  wait ; echo "RC terminated" \
# '


from sympy import true

import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory

#added to launch other launch files
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():

    # from nav2 bringup launch file
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')

    # ros2 launch rosbridge_server rosbridge_websocket_launch.xml
    bridge_bringup_dir = get_package_share_directory('rosbridge_server')
    bridge_launch_dir = os.path.join(bridge_bringup_dir, 'launch')
 
    # Get the text of the robot description URDF - robot_stat_publisher does not open a file
    with open('urdfs/rc.urdf','r') as infp:
        robot_desc = infp.read()

    # this causes the nodes to use the /clock time topic
    # which should process ros2 bag play .... --clock time properly
    use_sim_time:bool = True

    return launch.LaunchDescription([



        #### ROS2 packages

        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description':robot_desc,
                'use_sim_time':use_sim_time,
                }],
            remappings=[('/robot_description', '/rc/robot_description')],
        ),

        #######################################
        # Localization
  
        # Fuse wheel odom and imu data
        # Generate odom -> base_footprint TF
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='efk_local',
            parameters=[
                'config/efk_local_config.yaml',
                {'use_sim_time':use_sim_time},
            ],
            # output topic
            remappings=[("odometry/filtered", "odometry/local")],
        ),

        # # Fuse GPS wheel odom and imu data
        # # Generate map -> odom TF
        # launch_ros.actions.Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='efk_global',
        #     parameters=['config/efk_global_config.yaml'],
        #     # output topic
        #     remappings=[("odometry/filtered", "odometry/global")],
        # ),

        # # Transform GPS lat/lon to map coordinates
        # launch_ros.actions.Node(
        #     package='robot_localization',
        #     executable='navsat_transform_node',
        #     name='navsat_transform',
        #     parameters=['config/navsat_transform_config.yaml'],
        #     remappings=[
        #         # input topics
        #         ("imu/data", "imu"),
        #         ("gps/fix", "gps_nav"),
        #         ("odometry/filtered", "odometry/global"),
        #         # output topics
        #         ("gps/filtered", "gps/filtered"),
        #         ("odometry/gps", "odometry/gps"),
        #     ],
        # ),


    ])

