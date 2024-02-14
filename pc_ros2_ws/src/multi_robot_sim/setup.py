from setuptools import setup

package_name = 'multi_robot_sim'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/multi_robot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/multi_robot.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/robot1.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/robot2.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/multi_robot_map1.jpg']))
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
            'multi_robot_driver = multi_robot_sim.multi_robot_driver:main',
        ],
    },
)
