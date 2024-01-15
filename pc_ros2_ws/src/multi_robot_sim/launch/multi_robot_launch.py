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

    package_dir = get_package_share_directory('multi_robot_sim')
    robot1_description_path = os.path.join(package_dir, 'resource', 'robot1.urdf')
    robot2_description_path = os.path.join(package_dir, 'resource', 'robot2.urdf')

        
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')

    ## Robot frames and transforms nodes
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
    robot1_driver = WebotsController(
        robot_name='robot1',
        parameters=[
            {'robot_description': robot1_description_path},
        ]
    )
    robot2_driver = WebotsController(
        robot_name='robot2',
        parameters=[
            {'robot_description': robot2_description_path},
        ]
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
        
        joint_state_publisher,

        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])