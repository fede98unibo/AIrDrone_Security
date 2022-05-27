import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   
   config = os.path.join(
      get_package_share_directory('airdrone_navigation'),
      'config',
      'visual_tracker_params.yaml'
      )

   return LaunchDescription([
      Node(
         package='airdrone_navigation',
         executable='visual_tracker',
         name='visual_tracker',
         parameters=[config]
      )
   ])