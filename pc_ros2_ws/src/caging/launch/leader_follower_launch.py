import os
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from launch_ros.actions import Node
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    robot1_namespace = "robot1"
    robot2_namespace = "robot2"

    package_dir = get_package_share_directory('multi_robot_sim')

    ## Robot Control Nodes
    robot1_lqr_controller = Node(
        package='robot_control',
        executable='lqr_controller',
        parameters=[
            {'initial_pose': [-2.5,-0.5,0]},
            {'Q_factor': 0.1},
            {'R_factor': 0.001},
        ],
        namespace=robot1_namespace
    )

    follower_controller = Node(
        package='multi_robot_control',
        executable='leader_follower_controller',
        parameters=[
            {'robot1_initial_pose': [0.0,-0.5,0.0]},
            {'robot2_initial_pose': [0.0,0.5,0.0]},
            {'d_goal': 0.4},
            {'alpha_goal': 1.57},
            {'theta_f_goal': 1.57},
            {'K': [0.5,0.5,0.5]}
        ]
    )

    ## Vicon
    vicon_pose_publisher = Node(
        package='vicon',
        executable='multi_robot_pose_publisher'
    )



    return LaunchDescription([

        vicon_pose_publisher,

        #robot1_lqr_controller,
        follower_controller,
    ])