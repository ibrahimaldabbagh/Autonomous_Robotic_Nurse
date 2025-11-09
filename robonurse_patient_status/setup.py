from setuptools import setup

package_name = 'robonurse_patient_status'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/robonurse_patient_status']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/patient_status.launch.py']),
        ('share/' + package_name + '/config', ['config/status_map.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ibrahim Aldabbagh',
    maintainer_email='eng.ibrahim.aldabbagh@gmail.com',
    description='RoboNurse patient status node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'patient_status_node = robonurse_patient_status.nodes.patient_status_node:main'
        ],
    },
)
