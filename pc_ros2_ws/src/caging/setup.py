from setuptools import find_packages, setup

package_name = 'caging'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/multi_robot_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/leader_follower_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/lc_caging_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/lc_caging_launch_full.py']))

data_files.append(('share/' + package_name + '/resource', ['resource/caging_map.jpg']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eia',
    maintainer_email='alejotoro.o@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lc_caging_client = caging.lc_caging_client:main',
            'lc_caging_client_full = caging.lc_caging_client_full:main'
        ],
    },
)
