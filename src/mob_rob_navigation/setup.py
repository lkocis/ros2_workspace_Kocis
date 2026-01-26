from setuptools import find_packages, setup

package_name = 'mob_rob_navigation'

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
            'collision_avoidance = mob_rob_navigation.collision_avoidance:main',
            'nav_server = mob_rob_navigation.nav_action_server:main',
            'nav_client = mob_rob_navigation.nav_action_client:main',
        ],
    },
)
