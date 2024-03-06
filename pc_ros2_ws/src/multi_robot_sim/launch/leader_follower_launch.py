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
    robot1_description_path = os.path.join(package_dir, 'resource', 'robot1.urdf')
    robot2_description_path = os.path.join(package_dir, 'resource', 'robot2.urdf')

        
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

    follower_controller = Node(
        package='multi_robot_control',
        executable='leader_follower_controller',
        parameters=[
            {'robot1_initial_pose': [0.0,-0.5,0.0]},
            {'robot2_initial_pose': [0.0,0.5,0.0]},
            {'d_goal': 0.4},
            {'alpha_goal': 1.57},
            {'theta_f_goal': 1.57}
        ]
    )



    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='leader_follower.wbt',
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

        #robot1_lqr_controller,
        follower_controller,


        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])