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

    package_dir = get_package_share_directory('caging')

    map_path = os.path.join(package_dir, 'resource', 'caging_map.jpg')

    ## Vicon
    vicon_pose_publisher = Node(
        package='vicon',
        executable='object_pose_publisher'
    )

    object_pose_publisher = Node(
        package='multi_robot_sim',
        executable='object_pose_publisher',
        parameters=[
            {'d': 0.2},
        ],
        remappings=[
            ('/pose', '/robot1/pose'),
            ('/object/pose', '/vobject/pose')
        ]
    )

    ## Robot Control Nodes
    robot1_lqr_controller = Node(
        package='robot_control',
        executable='lc_lqr_controller',
        parameters=[
            {'initial_pose': [0.5,-2.5,1.57]},
            {'Q_factor': 0.1},
            {'R_factor': 0.8},
        ],
        namespace=robot1_namespace
    )

    robot2_lqr_controller = Node(
        package='robot_control',
        executable='lc_lqr_controller',
        parameters=[
            {'initial_pose': [-0.5,-2.5,1.57]},
            {'Q_factor': 0.1},
            {'R_factor': 0.8},
        ],
        namespace=robot2_namespace
    )

    robot2_caging_controller = Node(
        package='multi_robot_control',
        executable='lc_caging_controller',
        parameters=[
            {'d_goal': 0.5},
            {'alpha_goal': 0.0},
            {'K': [2.0,1.0,1.0]}
        ]
    )

    caging_lqr_controller = Node(
        package='robot_control',
        executable='lc_lqr_controller',
        name='caging_lqr_controller',
        parameters=[
            {'initial_pose': [0.0,0.0,0.0]},
            {'Q_factor': 0.1},
            {'R_factor': 0.8},
        ],
        remappings=[
            ('/pose', '/vobject/pose'),
            ('/cmd_vel', '/transformed_cmd_vel')
        ]
    )

    robot1_follow_trajectory_server = Node(
        package='robot_control',
        executable='follow_trajectory_server',
        namespace=robot1_namespace
    )

    robot2_follow_trajectory_server = Node(
        package='robot_control',
        executable='follow_trajectory_server',
        namespace=robot2_namespace
    )

    caging_follow_trajectory_server = Node(
        package='robot_control',
        executable='follow_trajectory_server',
    )

    robot1_control_gripper_server = Node(
        package="multi_robot_sim",
        executable="control_gripper_server",
        namespace=robot1_namespace,
    )

    robot2_control_gripper_server = Node(
        package="multi_robot_sim",
        executable="control_gripper_server",
        namespace=robot2_namespace,
    )

    cmd_vel_trans = Node(
        package='robot_control',
        executable='cmd_vel_trans',
        parameters=[
            {'d': 0.3},
        ],
        remappings=[
            ('/pose', '/robot1/pose'),
            ('/cmd_vel', '/robot1/cmd_vel')
        ]
    )

    ## Path planning
    path_planning_server = Node(
        package="path_planning",
        executable="obs_path_planning_server",
        parameters=[
            {"map_path": map_path}
        ],
    )

    ## Approach object server
    approach_object_server = Node(
        package="multi_robot_control",
        executable="approach_object_server",
    )

    return LaunchDescription([

        vicon_pose_publisher,
        object_pose_publisher,

        robot1_control_gripper_server,
        robot2_control_gripper_server,

        robot1_lqr_controller,
        robot2_lqr_controller,
        robot2_caging_controller,
        robot1_follow_trajectory_server,
        robot2_follow_trajectory_server,

        path_planning_server,

        approach_object_server,

        caging_lqr_controller,
        caging_follow_trajectory_server,
        cmd_vel_trans,

    ])