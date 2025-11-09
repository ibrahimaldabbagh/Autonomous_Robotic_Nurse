from setuptools import setup

package_name = 'robonurse_face_recognition'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/robonurse_face_recognition']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/face_params.yaml']),
        ('share/' + package_name + '/models', ['models/arcface.onnx']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ibrahim Aldabbagh',
    maintainer_email='eng.ibrahim.aldabbagh@gmail.com',
    description='RoboNurse face recognition package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'face_recognition_node = robonurse_face_recognition.nodes.face_recognition_node:main',
            'register_face = robonurse_face_recognition.register.register_face:main'
        ],
    },
)
