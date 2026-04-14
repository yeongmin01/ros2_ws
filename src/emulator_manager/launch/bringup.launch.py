from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # CAN Driver (can0)
        Node(
            package='can_driver',
            executable='can0_node',
            name='can0_node',
            output='screen'
        ),

        # CAN Driver (can1)
        Node(
            package='can_driver',
            executable='can1_node',
            name='can1_node',
            output='screen'
        ),

        # J1939 Parser
        Node(
            package='j1939_parser',
            executable='j1939_node',
            name='j1939_node',
            output='screen'
        ),

        # CANopen Parser
        Node(
            package='canopen_parser',
            executable='canopen_node',
            name='canopen_node',
            output='screen'
        ),

        # Motor Driver
        Node(
            package='motor_driver',
            executable='md_node',
            name='md_node',
            output='screen'
        ),

        # Steering Driver
        Node(
            package='steering_driver',
            executable='steering_node',
            name='steering_node',
            output='screen'
        ),

        # Emulator Manager
        Node(
            package='emulator_manager',
            executable='emulator_node',
            name='emulator_node',
            output='screen'
        ),
    ])