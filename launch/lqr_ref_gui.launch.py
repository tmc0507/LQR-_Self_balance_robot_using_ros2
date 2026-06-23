from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='selfbalancerobot',
            executable='lqr_ref_gui.py',
            name='lqr_ref_gui',
            output='screen',
            parameters=[
                {'target_node': '/self_balance_lqr_controller'},
                {'wheel_radius': 0.1},
                {'linear_limit': 1.0},
                {'omega_limit': 0.6},
                {'yaw_angle_limit_deg': 180.0},
            ],
        )
    ])
