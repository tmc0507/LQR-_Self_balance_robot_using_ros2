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
                {'phi_angle_limit_deg': 90.0},
                {'torque_plot_limit': 0.0},
                {'command_topic': '/wheel_effort_controller/commands'},
                {'imu_topic': '/imu'},
                {'joint_states_topic': '/joint_states'},
                {'left_joint': 'left_wheeL_joint'},
                {'right_joint': 'right_wheel_joint'},
            ],
        )
    ])
