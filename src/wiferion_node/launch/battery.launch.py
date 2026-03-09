from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('wiferion_node'),
        'config',
        'battery.yaml'
    ])

    return LaunchDescription([
        Node(
            package='wiferion_node',
            executable='wiferion_node',
            name='wiferion_battery',
            output='screen',
            parameters=[config]
        )
    ])
