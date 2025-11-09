from setuptools import setup

package_name = 'robonurse_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/robonurse_perception']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/perception.launch.py']),
        ('share/' + package_name + '/config', ['config/camera_params.yaml']),
        ('share/' + package_name + '/models', ['models/robonurse_yolo.onnx', 'models/classes.txt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ibrahim Aldabbagh',
    maintainer_email='eng.ibrahim.aldabbagh@gmail.com',
    description='RoboNurse perception: camera + YOLO',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'perception_camera_node = robonurse_perception.nodes.perception_camera_node:main'
        ],
    },
)
