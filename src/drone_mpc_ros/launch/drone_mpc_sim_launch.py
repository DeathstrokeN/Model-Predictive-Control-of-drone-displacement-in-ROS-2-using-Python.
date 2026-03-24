from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_mpc_ros',
            executable='reference_node',
            name='reference_node',
            output='screen',
        ),
        Node(
            package='drone_mpc_ros',
            executable='drone_simulator_node',
            name='drone_simulator_node',
            output='screen',
        ),
        Node(
            package='drone_mpc_ros',
            executable='mpc_controller_node',
            name='mpc_controller_node',
            output='screen',
        ),
    ])
