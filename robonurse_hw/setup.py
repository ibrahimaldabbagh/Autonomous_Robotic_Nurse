from setuptools import setup

package_name = 'robonurse_hw'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/robonurse_hw']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/motor_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ibrahim Aldabbagh',
    maintainer_email='eng.ibrahim.aldabbagh@gmail.com',
    description='RoboNurse hardware package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'motor_control_node = robonurse_hw.nodes.motor_control_node:main',
            'wheel_odometry_node = robonurse_hw.nodes.wheel_odometry_node:main'
        ],
    },
)
