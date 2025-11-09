import os
from glob import glob

package_name = 'video_stream_node'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ibrahim Aldabbagh',
    maintainer_email='eng.ibrahim.aldabbagh@gmail.com',
    description='Video streaming node for RoboNurse operator interface',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_stream_node = video_stream_node.video_stream_node:main',
            'rtsp_server = video_stream_node.rtsp_server:main',
        ],
    },
)