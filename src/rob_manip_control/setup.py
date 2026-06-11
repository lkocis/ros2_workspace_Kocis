from setuptools import find_packages, setup

package_name = 'rob_manip_control'

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
            'lana_drawer = rob_manip_control.lana_drawer:main',
            'path_visualizer = rob_manip_control.path_visualizer:main',
        ],
    },
)
