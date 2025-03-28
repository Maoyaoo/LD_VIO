from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_demo',
            executable='imu_driver',
            name='imu_driver',
            parameters=[{"~port_name":"/dev/ttyUSB0"}],
            output='both'
        )
    ])

