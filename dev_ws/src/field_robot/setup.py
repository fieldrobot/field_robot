import os
from glob import glob
from setuptools import setup

package_name = 'field_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/models/world', glob('models/world/*.sdf')),
        ('share/' + package_name + '/models/world', glob('models/world/*.config')),
        ('share/' + package_name + '/models/pumpkin', glob('models/pumpkin/*.sdf')),
        ('share/' + package_name + '/models/pumpkin', glob('models/pumpkin/*.config')),
        ('share/' + package_name + '/models/tb3/bases', glob('models/tb3/bases/*.stl')),
        ('share/' + package_name + '/models/tb3/wheels', glob('models/tb3/wheels/*.stl')),
        ('share/' + package_name + '/robot_description/tb3', glob('robot_description/tb3/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Simon Sure',
    maintainer_email='info@simonsure.com',
    description='This is the ROS2 code for the field_robot made mainly by Simon Sure and Erwin Kose.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawner = field_robot.robot_spawner:main',
        ],
    },
)
