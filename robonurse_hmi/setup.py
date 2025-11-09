from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robonurse_hmi'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install the launch file
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        # Assuming JSX files are hosted separately, but we include them here for completeness
        # If running a local web server (like Nginx/Apache) on the robot, these files would go there.
        (os.path.join('share', package_name, 'web'), glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@ros.org',
    description='ROS 2 nodes and web interfaces for the RoboNurse HMI system.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_bridge_node = robonurse_hmi.teleop_bridge_node:main',
            'task_manager_node = robonurse_hmi.task_manager_node:main',
            'actuator_control_node = robonurse_hmi.actuator_control_node:main',
            # Add other necessary nodes here (e.g., face_recognition_node, nav_monitor_node)
        ],
    },
)