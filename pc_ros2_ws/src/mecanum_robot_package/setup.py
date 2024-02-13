from setuptools import setup

package_name = 'mecanum_robot_package'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/path_planning_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/mecanum_robot.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/mecanum_robot_2.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/robot.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/test_map2.jpg']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user.name@mail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_robot_driver = mecanum_robot_package.my_robot_driver:main',
            'odometry_publisher = mecanum_robot_package.odometry_publisher:main',
            'robot_navigation = mecanum_robot_package.robot_navigation:main'
        ],
    },
)
