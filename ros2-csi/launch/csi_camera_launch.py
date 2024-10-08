from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='csi_camera',
            executable='csi_camera_node',
            name='csi_camera_node',
            output='screen',
            parameters=[
                # Add any ROS parameters here if needed
            ],
        ),
    ])
