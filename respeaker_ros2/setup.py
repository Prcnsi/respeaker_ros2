from setuptools import setup

package_name = 'respeaker_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jiseon Kim',
    maintainer_email='ji.dozin@gmail.com',
    description='ReSpeaker ROS 2 audio and DOA publisher',
    license='MIT',
    entry_points={
        'console_scripts': [
            'audio_doa_publisher = respeaker_ros2.audio_doa_publisher:main',
        ],
    },
)
