from setuptools import setup

setup(
    name='respeaker_ros2',
    version='0.0.1',
    packages=['respeaker_ros2'],
    install_requires=[
        'rclpy',
        'pyusb',
        'pyaudio',
        'numpy',
        'click',
    ],
    entry_points={
        'console_scripts': [
            'audio_with_doa_node = respeaker_ros2.audio_with_doa_publisher:main',
        ],
    },
)
