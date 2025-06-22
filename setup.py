from setuptools import setup, find_packages

package_name = 'respeaker_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    zip_safe=True,
    author='Jiseon Kim',
    author_email='ji.dozin@gmail.com',
    maintainer='Jiseon Kim',
    maintainer_email='ji.dozin@gmail.com',
    description='ROS 2 package for ReSpeaker USB Microphone Array v2.0 to publish audio data and DOA.',
    license='Apache 2.0',
    tests_require=['pytest'],
)
