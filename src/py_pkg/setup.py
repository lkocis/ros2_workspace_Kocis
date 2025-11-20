from setuptools import find_packages, setup
import os
import glob

package_name = 'py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lana',
    maintainer_email='lana@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': ["my_first_node_py = py_pkg.my_first_node_py:main",
                            "complex_pub = py_pkg.complex_publisher:main",
                            "complex_sub = py_pkg.complex_subscriber:main",
                            "countwords_srv = py_pkg.countwords_service:main",
                            "countwords_cli = py_pkg.countwords_client:main",
                            "countuntil_act = py_pkg.countuntil_action:main",
                            "countuntil_cli = py_pkg.countuntil_client:main",
                            "draw_house = py_pkg.draw_house:main",
                            "draw_lana = py_pkg.draw_lana:main",
                            "znamenitosti_pub_sub = py_pkg.znamenitosti_pub_sub:main",
                            "tf_listen = py_pkg.tf_listen:main",
                            "tf_pub = py_pkg.tf_pub:main"
        ],
    },
)
