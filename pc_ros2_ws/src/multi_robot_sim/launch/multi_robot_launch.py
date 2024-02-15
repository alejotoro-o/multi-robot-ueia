import os
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from launch_ros.actions import Node
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot1_namespace = "robot1"
    robot2_namespace = "robot2"

    package_dir = get_package_share_directory('multi_robot_sim')
    robot1_description_path = os.path.join(package_dir, 'resource', 'robot1.urdf')
    robot2_description_path = os.path.join(package_dir, 'resource', 'robot2.urdf')

    map_path = os.path.join(package_dir, 'resource', 'multi_robot_map1.jpg')

        
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')

    ## Robot frames and transforms nodes
    robot1_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'source_list':["wheels_encoders"]}],
        namespace=robot1_namespace
    )

    robot2_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'source_list':["wheels_encoders"]}],
        namespace=robot2_namespace
    )

    ## Webots and Robot Nodes
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        mode=mode,
        ros2_supervisor=True,
    )
    robot1_driver = WebotsController(
        robot_name='robot1',
        parameters=[
            {'robot_description': robot1_description_path},
        ],
        namespace=robot1_namespace
    )
    robot2_driver = WebotsController(
        robot_name='robot2',
        parameters=[
            {'robot_description': robot2_description_path},
        ],
        namespace=robot2_namespace
    )

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

    robot2_lqr_controller = Node(
        package='robot_control',
        executable='lqr_controller',
        parameters=[
            {'initial_pose': [-2.5,0.5,0]},
            {'Q_factor': 0.1},
            {'R_factor': 0.001},
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
        executable="path_planning_server",
        parameters=[
            {"map_path": map_path}
        ],
        namespace=robot1_namespace
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='multi_robot.wbt',
            description='Choose one of the world files'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode'
        ),
        webots,
        webots._supervisor,

        robot1_driver,
        robot2_driver,
        
        robot1_joint_state_publisher,
        robot2_joint_state_publisher,

        robot1_lqr_controller,
        robot2_lqr_controller,
        robot1_follow_trajectory_server,
        robot2_follow_trajectory_server,

        path_planning_server,

        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])