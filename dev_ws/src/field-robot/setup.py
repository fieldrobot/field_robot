import os
from glob import glob
from setuptools import setup

package_name = 'field-robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.xml')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/models', glob('models/*.xacro')),
        ('share/' + package_name + '/models/world', glob('models/world/*.sdf')),
        ('share/' + package_name + '/models/world', glob('models/world/*.config')),
        ('share/' + package_name + '/models/robot', glob('models/robot/*.sdf')),
        ('share/' + package_name + '/models/robot', glob('models/robot/*.config')),
        ('share/' + package_name + '/models/pumpkin', glob('models/pumpkin/*.sdf')),
        ('share/' + package_name + '/models/pumpkin', glob('models/pumpkin/*.config')),
        ('share/' + package_name + '/models/turtlebot3_world', glob('models/turtlebot3_world/*.sdf')),
        ('share/' + package_name + '/models/turtlebot3_world', glob('models/turtlebot3_world/*.config')),
        ('share/' + package_name + '/models/turtlebot3_world/meshes', glob('models/turtlebot3_world/meshes/*')),
        ('share/' + package_name + '/models/turtlebot3_burger', glob('models/turtlebot3_burger/*.sdf')),
        ('share/' + package_name + '/models/turtlebot3_burger', glob('models/turtlebot3_burger/*.config')),
        ('share/' + package_name + '/models/turtlebot3_burger/meshes', glob('models/turtlebot3_burger/meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Simon Sure',
    maintainer_email='info@simonsure.com',
    description='This is the ROS2 code for the field-robot made mainly by Simon Sure and Erwin Kose.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
