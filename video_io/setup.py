from setuptools import setup

package_name = 'video_io'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Simone Giordani',
    maintainer_email='simone.giorani2@studio.unibo.it',
    description='Video input and output',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'video = video_io.video_publisher:main',
            'visualize = video_io.visualize_detections:main'
        ],
    },
)
