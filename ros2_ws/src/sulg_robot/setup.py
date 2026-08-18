import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sulg_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Azmain',
    maintainer_email='azmain@example.com',
    description='ROS 2 Jazzy Autonomous Robotics Navigation Suite for Sulg',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sulg_hardware_bridge = sulg_robot.hardware_bridge:main',
            'sulg_vision_detector = sulg_robot.vision_detector:main',
            'sulg_autonomous_nav = sulg_robot.autonomous_nav:main',
            'sulg_web_bridge = sulg_robot.web_bridge:main',
        ],
    },
)
