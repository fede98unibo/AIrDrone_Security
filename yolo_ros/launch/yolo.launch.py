from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    launch_description = [] # Append here your nodes

    launch_description.append(
        Node(
            package='yolo_ros',
            executable='yolo',
            parameters=[{
                        # Following params
                        'distance_treshold': 250.0,
                        'history_timeout': 5.0

                        # Yolo params 

                        }],
            output='screen',
        ))

    return LaunchDescription(launch_description)

def main():
   generate_launch_description()

if __name__ == "__main__":
    main()