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
        ('share/' + package_name + '/models/maize_01/meshes', glob('models/maize_01/meshes/maize_01.dae')),
        ('share/' + package_name + '/models/maize_01/textures', glob('models/maize_01/textures/maize_01.png')),
        ('share/' + package_name + '/models/maize_01', glob('models/maize_01/model.config')),
        ('share/' + package_name + '/models/maize_02/meshes', glob('models/maize_02/meshes/maize_02.dae')),
        ('share/' + package_name + '/models/maize_02/textures', glob('models/maize_02/textures/maize_02.png')),
        ('share/' + package_name + '/models/maize_02', glob('models/maize_02/model.config')),
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
            'robot_spawner = field_robot.robot_spawner:main',
        ],
    },
)
