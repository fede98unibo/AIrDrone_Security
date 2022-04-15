import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()

    surf_config = os.path.join(
    get_package_share_directory('airdrone_navigation'),
    'config',
    'target_detector_params.yaml'
    )

    node=Node(
        package = 'airdrone_navigation',
        name = 'target_detector',
        executable = 'SurfDetector',
        parameters = [surf_config]
    )

    
    
    ld.add_action(node)
    return ld