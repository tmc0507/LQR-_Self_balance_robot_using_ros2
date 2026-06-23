from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():

    
    robot_path = get_package_share_directory('selfbalancerobot')
    
    parent_share_path = os.path.dirname(robot_path)


    urdf_file = os.path.join(robot_path, 'urdf', 'selfbalancerobot.urdf')
    world_file = os.path.join(robot_path, 'empty.sdf')
    controller_config = os.path.join(robot_path, 'config', 'controllers.yaml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read().replace(
            '__ROS2_CONTROL_CONFIG__',
            controller_config
        )

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=parent_share_path
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

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.1'
        ],
        output='screen'
    )

    return LaunchDescription([

        set_gz_resource_path,
        robot_state_publisher,
        
        ExecuteProcess(
            cmd=['gz', 'sim', world_file],
            output='screen'
        ),

        TimerAction(period=3.0, actions=[spawn_robot])

    ])
