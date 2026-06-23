import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('selfbalancerobot')

    urdf_file = os.path.join(pkg, 'urdf', 'selfbalancerobot.urdf')
    world_file = os.path.join(pkg, 'empty.sdf')
    controller_config = os.path.join(pkg, 'config', 'controllers.yaml')
    bridge_config = os.path.join(pkg, 'config', 'bridge_config.yaml')
    models_dir = os.path.join(pkg, 'models')

    with open(urdf_file, 'r') as f:
        robot_description = f.read().replace(
            '__ROS2_CONTROL_CONFIG__',
            controller_config
        )

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{os.path.dirname(pkg)}:{models_dir}"
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ],
        output='screen'
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'selfbalancerobot',
            '-x', '0',
            '-y', '0',
            '-z', '0.1',
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--param-file',
            controller_config,
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        robot_state_publisher,
        gz_sim,
        TimerAction(period=3.0, actions=[spawn_robot]),
        TimerAction(period=5.0, actions=[bridge]),
        TimerAction(
            period=8.0,
            actions=[
                joint_state_broadcaster,
                diff_drive_controller,
            ]
        ),
    ])
