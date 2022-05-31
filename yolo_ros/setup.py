from setuptools import setup
from glob import glob

package_name = 'yolo_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Simone Giordani',
    maintainer_email='simone.giorani2@studio.unibo.it',
    description='Tensorflow YoloV4 models in ROS2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'yolo = yolo_ros.yolo_node:main',
        ],
    },
)
