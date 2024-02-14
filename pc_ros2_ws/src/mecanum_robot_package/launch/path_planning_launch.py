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

    package_dir = get_package_share_directory('mecanum_robot_package')
    robot_description_path = os.path.join(package_dir, 'resource', 'robot.urdf')
    with open(robot_description_path, 'r') as desc:
        robot_description = desc.read()

    map_path = os.path.join(package_dir, 'resource', 'test_map2.jpg')
        
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')

    ## Robot frames and transforms nodes
    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }],
        arguments=[robot_description_path],
    )
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'source_list':["wheels_encoders"]}],
    )

    ## Webots and Robot Nodes
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        mode=mode,
        ros2_supervisor=True,
    )
    my_robot_driver = WebotsController(
        robot_name='robot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )
    odometry_publisher = Node(
        package='mecanum_robot_package',
        executable='odometry_publisher',
    )

    ## Robot control
    lqr_controller = Node(
        package='robot_control',
        executable='lqr_controller',
        parameters=[
            {'initial_pose': [0,2.5,-1.57]}
        ]
    )

    follow_trajectory_server = Node(
        package='robot_control',
        executable='follow_trajectory_server'
    )

    ## Path planning
    path_planning_server = Node(
        package="path_planning",
        executable="path_planning_server",
        parameters=[
            {"map_path": map_path}
        ]
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='mecanum_robot_2.wbt',
            description='Choose one of the world files'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode'
        ),
        webots,
        webots._supervisor,
        my_robot_driver,
        odometry_publisher,
        
        footprint_publisher,
        robot_state_publisher,
        joint_state_publisher,

        lqr_controller,
        follow_trajectory_server,

        path_planning_server,

        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])