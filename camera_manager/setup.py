from setuptools import setup

package_name = 'camera_manager'

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
    maintainer='fede',
    maintainer_email='fede@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_server = camera_manager.camera_manager_server:main',
            'camera_client = camera_manager.camera_manager_client:main',
            'camera_action_client = camera_manager.camera_manager_action_client:main',
        ],
    },
)
