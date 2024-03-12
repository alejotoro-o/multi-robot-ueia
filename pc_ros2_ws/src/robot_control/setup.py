from setuptools import find_packages, setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eia',
    maintainer_email='eia@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lqr_controller = robot_control.lqr_controller:main',
            'nav_to_pose_client = robot_control.robot_control_client:main',

            'follow_path_server = robot_control.follow_path:main',
            'follow_path_client = robot_control.follow_path_client:main',

            'follow_trajectory_server = robot_control.follow_trajectory:main',
            'follow_trajectory_client = robot_control.follow_trajectory_client:main',

            'cmd_vel_trans = robot_control.cmd_vel_trans:main',

            "lc_lqr_controller = robot_control.lc_lqr_controller:main",
            "lc_cmd_vel_trans = robot_control.lc_cmd_vel_trans:main"
        ],
    },
)
