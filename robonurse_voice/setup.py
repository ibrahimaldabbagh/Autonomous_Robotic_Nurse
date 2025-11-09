from setuptools import find_packages, setup

package_name = 'robonurse_voice'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'vosk', 'pyaudio', 'gtts'], 
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='Speech Recognition and TTS nodes for RoboNurse',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speech_asr_node = robonurse_voice.speech_asr_node:main',
            'speech_tts_node = robonurse_voice.speech_tts_node:main',
        ],
    },
)