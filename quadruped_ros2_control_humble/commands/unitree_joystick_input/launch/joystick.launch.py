from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('network_interface', description='Confirmed robot network interface'),
        DeclareLaunchArgument('domain', description='Confirmed robot DDS domain'),
        DeclareLaunchArgument('read_only', default_value='true', description='Observe keys without publishing control_input'),
        DeclareLaunchArgument('stop_only', default_value='true', description='Decode only stop commands; zero all axes'),
        DeclareLaunchArgument('state_source', default_value='lowstate', description='lowstate or wirelesscontroller; no automatic fallback'),
        Node(
            package='unitree_joystick_input',
            executable='joystick_input',
            name='joystick_input_node',
            parameters=[
                {
                    'network_interface': LaunchConfiguration('network_interface'),
                    'domain': ParameterValue(LaunchConfiguration('domain'), value_type=int),
                    'read_only': ParameterValue(LaunchConfiguration('read_only'), value_type=bool),
                    'stop_only': ParameterValue(LaunchConfiguration('stop_only'), value_type=bool),
                    'state_source': LaunchConfiguration('state_source'),
                }
            ],
        )
    ])
