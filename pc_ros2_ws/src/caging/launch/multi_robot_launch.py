import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    robot1_namespace = "robot1"
    robot2_namespace = "robot2"

    package_dir = get_package_share_directory('caging')

    map_path = os.path.join(package_dir, 'resource', 'caging_map.jpg')

    ## Vicon
    vicon_pose_publisher = Node(
        package='vicon',
        executable='multi_robot_pose_publisher'
    )

    ## Robot Control Nodes
    robot1_lqr_controller = Node(
        package='robot_control',
        executable='lqr_controller',
        parameters=[
            {'initial_pose': [0.5,2.5,-1.57]},
            {'Q_factor': 0.1},
            {'R_factor': 0.8},
        ],
        namespace=robot1_namespace
    )

    robot2_lqr_controller = Node(
        package='robot_control',
        executable='lqr_controller',
        parameters=[
            {'initial_pose': [-0.5,2.5,-1.57]},
            {'Q_factor': 0.1},
            {'R_factor': 0.8},
        ],
        namespace=robot2_namespace
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

        robot1_lqr_controller,
        robot2_lqr_controller,
        robot1_follow_trajectory_server,
        robot2_follow_trajectory_server,

        path_planning_server,

        approach_object_server,
    ])