import os
from setuptools import find_packages, setup

package_name = 'robot_nav_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/nav2_mob_rob.launch.xml']),
        (os.path.join('share', package_name, 'config'), ['config/nav2_params.yaml', 'config/gz_bridge.yaml']),
        (os.path.join('share', package_name, 'maps'), ['maps/depot.yaml', 'maps/depot.pgm']),
        (os.path.join('share', package_name, 'rviz'), ['rviz/navigation.rviz']),
        (os.path.join('share', package_name, 'worlds'), ['worlds/depot.sdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lana',
    maintainer_email='lana.kocis@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'goal_manager = robot_nav_pkg.goal_manager:main',
            'current_point_manager = robot_nav_pkg.current_point_manager:main',
        ],
    },
)
