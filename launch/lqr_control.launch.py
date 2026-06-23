import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('selfbalancerobot')

    gazebo_effort_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'gazebo_effort_control.launch.py')
        )
    )

    lqr_controller = Node(
        package='selfbalancerobot',
        executable='lqr_controller.py',
        name='self_balance_lqr_controller',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'theta_dot_ref': 0.0},
            {'phi_dot_ref': 0.0},
            {'phi_dot_ref_slew_rate': 0.8},
            {'yaw_reference_mode': 'rate'},
            {'phi_ref_kp': 1.8},
            {'phi_dot_ref_limit': 0.35},
            {'reset_yaw_on_omega_stop': True},
            {'omega_stop_threshold': 0.02},
            {'yaw_rate_brake_gain': 0.8},
            {'yaw_rate_brake_limit': 1.0},
            {'yaw_torque_feedforward': 1.6},
            {'yaw_torque_feedforward_deadband': 0.03},
            {'yaw_torque_max': 2.2},
            {'yaw_torque_sign': 1.0},
            {'hold_initial_position': True},
            {'hold_initial_yaw': True},
            {'torque_limit': 15.0},
            {'wheel_velocity_limit': 16.0},
            {'overspeed_brake_gain': 4.0},
            {'torque_slew_rate_limit': 45.0},
            {'q_diag': [0.01, 1.0, 40.0, 8.0, 1.5, 4.0]},
            {'r_diag': [10.0, 10.0]},
            {'max_tilt_rad': 0.7},
            {'pitch_sign': 1.0},
            {'torque_sign': 1.0},
            {'invert_left_wheel': False},
            {'invert_right_wheel': False},
        ],
    )

    return LaunchDescription([
        gazebo_effort_launch,
        TimerAction(period=10.0, actions=[lqr_controller]),
    ])
