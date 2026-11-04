import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    package_description = context.launch_configurations['pkg_description']
    network_interface = context.launch_configurations['network_interface']
    domain = context.launch_configurations['domain']
    if network_interface == 'lo' or not network_interface:
        raise ValueError('Real Go2 launch requires an explicitly confirmed non-loopback interface')
    pkg_path = os.path.join(get_package_share_directory(package_description))

    xacro_file = os.path.join(pkg_path, 'xacro', 'robot.xacro')
    robot_description = xacro.process_file(xacro_file, mappings={
        'network_interface': network_interface, 'domain': domain,
    }).toxml()

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(package_description),
            "config",
            "robot_control.yaml",
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'publish_frequency': 20.0,
                'use_tf_static': True,
                'robot_description': robot_description,
                'ignore_timestamp': True
            }
        ],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers, {
            'rl_quadruped_controller.runtime_mode': 'hardware',
        }],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both",
    )

    joint_state_publisher = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    imu_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rl_quadruped_controller", "--controller-manager", "/controller_manager"],
    )

    remote_input = Node(
        package='unitree_joystick_input',
        executable='joystick_input',
        name='go2_remote_input',
        parameters=[{
            'network_interface': network_interface,
            'domain': int(domain),
            'read_only': False,
            # Until standing/motion commissioning explicitly enables other inputs.
            'stop_only': True,
            'state_source': 'lowstate',
        }],
        output='screen',
    )

    return [
        remote_input,
        robot_state_publisher,
        controller_manager,
        joint_state_publisher,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_publisher,
                on_exit=[imu_sensor_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=imu_sensor_broadcaster,
                on_exit=[controller],
            )
        ),
    ]


def generate_launch_description():
    pkg_description = DeclareLaunchArgument(
        'pkg_description',
        default_value='go2_description',
        description='package for robot description'
    )
    network_interface = DeclareLaunchArgument(
        'network_interface',
        description='Confirmed network interface for the real Go2 DDS connection'
    )

    return LaunchDescription([
        pkg_description,
        network_interface,
        DeclareLaunchArgument('domain', description='Confirmed DDS domain for the real Go2'),
        # A test fault inherited from a developer shell must fail closed, never arm on hardware.
        SetEnvironmentVariable('ABS_SIMULATION_TEST', '0'),
        OpaqueFunction(function=launch_setup),
    ])
