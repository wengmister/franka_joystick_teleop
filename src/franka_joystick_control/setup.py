from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'franka_joystick_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zhengyang Kris Weng',
    maintainer_email='wengmister@gmail.com',
    description='Xbox joystick controller for Franka FER',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'franka_joystick_publisher = franka_joystick_control.franka_joystick_publisher:main',
        ]
    },
)
