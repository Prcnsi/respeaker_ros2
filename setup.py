from setuptools import setup

package_name = 'respeaker_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Jiseon Ki',
    author_email='jiseon.ki@example.com',
    maintainer='Jiseon Ki',
    maintainer_email='jiseon.ki@example.com',
    description='ROS 2 package for ReSpeaker USB Microphone Array v2.0 to publish audio data and DOA.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'audio_with_doa_publisher = respeaker_ros2.audio_with_doa_publisher:main',
        ],
    },
)
