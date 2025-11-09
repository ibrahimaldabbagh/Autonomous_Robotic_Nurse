from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robonurse_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install resource file
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files (the navigation_node launch script)
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[py]*'))),
        # Install parameter files
        (os.path.join('share', package_name, 'params'), glob('params/*')),
        # Install map files (You must place your map.yaml and map.pgm here)
        (os.path.join('share', package_name, 'maps'), glob('maps/*'))
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='Navigation stack configuration and local safety planner for RoboNurse',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # The actual executable for your custom safety planner
            'local_planner_node = robonurse_nav.local_planner_node:main',
        ],
        'launch.frontend.launch_extension': [
            # Allows running the top-level navigation launch file
            'launch_entry = robonurse_nav.navigation_node:generate_launch_description',
        ],
    },
)