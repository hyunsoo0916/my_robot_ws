from setuptools import setup
import os
from glob import glob

package_name = 'my_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyunsoo',
    maintainer_email='hyunsoo@todo.todo',
    description='My TurtleBot3 URDF test package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'sensor_fusion_node = my_robot.sensor_fusion_node:main',
        ],
    }
)