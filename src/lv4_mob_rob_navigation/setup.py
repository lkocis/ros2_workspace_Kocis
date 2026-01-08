from setuptools import find_packages, setup

package_name = 'lv4_mob_rob_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mob_rob_navigation_launch.xml']),
        ('share/' + package_name + '/config', ['config/gz_bridge.yaml']),
        ('share/' + package_name + '/world', ['world/depot.sdf']),
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
            "navigator = lv4_mob_rob_navigation.navigator:main",
            "obstacle_detector = lv4_mob_rob_navigation.obstacle_detector:main",
            "keyboard_navigation = lv4_mob_rob_navigation.keyboard_navigation:main",
        ],
    },
)
