#!/usr/bin/python3
import math

import numpy as np
from scipy.linalg import solve_continuous_are

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray


def quaternion_to_euler(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def wrap_to_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def compute_lqr_gain(q_diag, r_diag):
    m = 4.25
    m_body = 26.0 - 4.25 * 2.0
    rw = 0.200 / 2.0
    wheel_separation = 0.32
    length_to_com = 0.14
    gravity = 9.81

    jw = 0.0075
    jm = 0.0074
    jpsi = 0.4533
    jphi = 0.4021

    fm = 0.01
    fw = 0.02

    k = wheel_separation / (2.0 * rw)
    a = (2.0 * m + m_body) * rw**2 + 2.0 * jw + 2.0 * jm
    b = m_body * length_to_com * rw - 2.0 * jm
    c = m_body * length_to_com**2 + jpsi + 2.0 * jm
    d = (m * wheel_separation**2) / 2.0 + jphi + (
        wheel_separation**2 / (2.0 * rw**2)
    ) * (jw + jm)

    delta = a * c - b**2

    a22 = (-2.0 * c * (fm + fw) - 2.0 * b * fm) / delta
    a23 = -(b * m_body * gravity * length_to_com) / delta
    a24 = (2.0 * fm * (c + b)) / delta

    a42 = (2.0 * b * (fm + fw) + 2.0 * a * fm) / delta
    a43 = (a * m_body * gravity * length_to_com) / delta
    a44 = -(2.0 * fm * (a + b)) / delta

    a66 = -(2.0 * k**2 * (fm + fw)) / d

    a_matrix = np.array(
        [
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, a22, a23, a24, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, a42, a43, a44, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, a66],
        ],
        dtype=float,
    )

    b21 = (c + b) / delta
    b41 = -(a + b) / delta
    b61 = k / d

    b_matrix = np.array(
        [
            [0.0, 0.0],
            [b21, b21],
            [0.0, 0.0],
            [b41, b41],
            [0.0, 0.0],
            [b61, -b61],
        ],
        dtype=float,
    )

    q_matrix = np.diag(q_diag)
    r_matrix = np.diag(r_diag)

    s_matrix = solve_continuous_are(a_matrix, b_matrix, q_matrix, r_matrix)
    return np.linalg.solve(r_matrix, b_matrix.T @ s_matrix)


class LqrController(Node):
    def __init__(self):
        super().__init__('self_balance_lqr_controller')

        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('command_topic', '/wheel_effort_controller/commands')
        self.declare_parameter('left_joint', 'left_wheeL_joint')
        self.declare_parameter('right_joint', 'right_wheel_joint')
        self.declare_parameter('control_rate_hz', 100.0)
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('torque_limit', 15.0)
        self.declare_parameter('wheel_velocity_limit', 16.0)
        self.declare_parameter('overspeed_brake_gain', 4.0)
        self.declare_parameter('torque_slew_rate_limit', 80.0)
        self.declare_parameter('max_tilt_rad', 0.7)
        self.declare_parameter('q_diag', [0.01, 10.0, 40.0, 8.0, 1.0, 2.0])
        self.declare_parameter('r_diag', [10.0, 10.0])
        self.declare_parameter('theta_dot_ref', 0.0)
        self.declare_parameter('phi_dot_ref', 0.0)
        self.declare_parameter('phi_dot_ref_slew_rate', 0.6)
        self.declare_parameter('yaw_reference_mode', 'rate')
        self.declare_parameter('phi_ref_delta_request', 0.0)
        self.declare_parameter('phi_ref_delta_request_id', 0)
        self.declare_parameter('phi_ref_kp', 1.2)
        self.declare_parameter('phi_dot_ref_limit', 0.5)
        self.declare_parameter('reset_reference_request', False)
        self.declare_parameter('reset_yaw_on_omega_stop', True)
        self.declare_parameter('omega_stop_threshold', 0.02)
        self.declare_parameter('yaw_rate_brake_gain', 0.0)
        self.declare_parameter('yaw_rate_brake_limit', 0.0)
        self.declare_parameter('yaw_torque_feedforward', 0.0)
        self.declare_parameter('yaw_torque_feedforward_deadband', 0.03)
        self.declare_parameter('yaw_torque_max', 2.2)
        self.declare_parameter('yaw_torque_sign', 1.0)
        self.declare_parameter('hold_initial_position', True)
        self.declare_parameter('hold_initial_yaw', True)
        self.declare_parameter('pitch_sign', 1.0)
        self.declare_parameter('torque_sign', 1.0)
        self.declare_parameter('invert_left_wheel', False)
        self.declare_parameter('invert_right_wheel', False)

        self.left_joint = self.get_parameter('left_joint').value
        self.right_joint = self.get_parameter('right_joint').value
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.torque_limit = float(self.get_parameter('torque_limit').value)
        self.wheel_velocity_limit = float(self.get_parameter('wheel_velocity_limit').value)
        self.overspeed_brake_gain = float(self.get_parameter('overspeed_brake_gain').value)
        self.torque_slew_rate_limit = float(self.get_parameter('torque_slew_rate_limit').value)
        self.max_tilt_rad = float(self.get_parameter('max_tilt_rad').value)
        self.reset_yaw_on_omega_stop = bool(self.get_parameter('reset_yaw_on_omega_stop').value)
        self.omega_stop_threshold = float(self.get_parameter('omega_stop_threshold').value)
        self.yaw_rate_brake_gain = float(self.get_parameter('yaw_rate_brake_gain').value)
        self.yaw_rate_brake_limit = float(self.get_parameter('yaw_rate_brake_limit').value)
        self.yaw_torque_feedforward = float(self.get_parameter('yaw_torque_feedforward').value)
        self.yaw_torque_feedforward_deadband = float(
            self.get_parameter('yaw_torque_feedforward_deadband').value
        )
        self.yaw_torque_max = float(self.get_parameter('yaw_torque_max').value)
        self.yaw_torque_sign = float(self.get_parameter('yaw_torque_sign').value)
        self.phi_dot_ref_slew_rate = float(self.get_parameter('phi_dot_ref_slew_rate').value)
        self.phi_ref_kp = float(self.get_parameter('phi_ref_kp').value)
        self.phi_dot_ref_limit = float(self.get_parameter('phi_dot_ref_limit').value)
        self.hold_initial_position = bool(self.get_parameter('hold_initial_position').value)
        self.hold_initial_yaw = bool(self.get_parameter('hold_initial_yaw').value)
        self.pitch_sign = float(self.get_parameter('pitch_sign').value)
        self.torque_sign = float(self.get_parameter('torque_sign').value)
        self.invert_left_wheel = bool(self.get_parameter('invert_left_wheel').value)
        self.invert_right_wheel = bool(self.get_parameter('invert_right_wheel').value)

        q_diag = list(self.get_parameter('q_diag').value)
        r_diag = list(self.get_parameter('r_diag').value)
        if len(q_diag) != 6:
            raise ValueError('q_diag must contain 6 values: theta, theta_dot, psi, psi_dot, phi, phi_dot')
        if len(r_diag) != 2:
            raise ValueError('r_diag must contain 2 values: tau_l, tau_r')

        self.k_gain = compute_lqr_gain(q_diag, r_diag)
        self.get_logger().info(f'LQR q_diag={q_diag}, r_diag={r_diag}')
        self.get_logger().info(f'LQR K gain:\n{self.k_gain}')

        self.imu_msg = None
        self.joint_msg = None
        self.theta_ref = 0.0
        self.phi_ref = 0.0
        self.references_initialized = False
        self.last_time = None
        self.last_torque = np.zeros(2)
        self.previous_phi_dot_ref = 0.0
        self.filtered_phi_dot_ref = 0.0
        self.last_phi_ref_delta_request_id = int(
            self.get_parameter('phi_ref_delta_request_id').value
        )
        self.last_yaw_debug_time = 0.0

        self.command_pub = self.create_publisher(
            Float64MultiArray,
            self.get_parameter('command_topic').value,
            10,
        )
        self.create_subscription(
            Imu,
            self.get_parameter('imu_topic').value,
            self.imu_callback,
            10,
        )
        self.create_subscription(
            JointState,
            self.get_parameter('joint_states_topic').value,
            self.joint_state_callback,
            10,
        )

        control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.timer = self.create_timer(1.0 / control_rate_hz, self.control_step)

    def imu_callback(self, msg):
        self.imu_msg = msg

    def joint_state_callback(self, msg):
        self.joint_msg = msg

    def get_joint_state(self, joint_name):
        if self.joint_msg is None or joint_name not in self.joint_msg.name:
            return None, None

        index = self.joint_msg.name.index(joint_name)
        position = self.joint_msg.position[index] if index < len(self.joint_msg.position) else 0.0
        velocity = self.joint_msg.velocity[index] if index < len(self.joint_msg.velocity) else 0.0
        return position, velocity

    def publish_torque(self, left_torque, right_torque):
        msg = Float64MultiArray()
        msg.data = [float(left_torque), float(right_torque)]
        self.command_pub.publish(msg)

    def limit_wheel_speed(self, torque, velocity):
        if velocity > self.wheel_velocity_limit:
            brake = -self.overspeed_brake_gain * (velocity - self.wheel_velocity_limit)
            return min(torque, brake)
        if velocity < -self.wheel_velocity_limit:
            brake = -self.overspeed_brake_gain * (velocity + self.wheel_velocity_limit)
            return max(torque, brake)
        return torque

    def limit_torque_slew_rate(self, torque, dt):
        if dt <= 0.0:
            return torque
        max_delta = self.torque_slew_rate_limit * dt
        delta = np.clip(torque - self.last_torque, -max_delta, max_delta)
        return self.last_torque + delta

    def limit_reference_slew_rate(self, target, current, max_rate, dt):
        if dt <= 0.0 or max_rate <= 0.0:
            return target
        max_delta = max_rate * dt
        return current + float(np.clip(target - current, -max_delta, max_delta))

    def control_step(self):
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            self.publish_torque(0.0, 0.0)
            return

        dt = max((now - self.last_time).nanoseconds * 1e-9, 0.0)
        self.last_time = now

        if self.imu_msg is None or self.joint_msg is None:
            self.publish_torque(0.0, 0.0)
            return

        left_pos, left_vel = self.get_joint_state(self.left_joint)
        right_pos, right_vel = self.get_joint_state(self.right_joint)
        if left_pos is None or right_pos is None:
            self.publish_torque(0.0, 0.0)
            self.get_logger().warn('Waiting for wheel joint states...', throttle_duration_sec=2.0)
            return

        if self.invert_left_wheel:
            left_pos = -left_pos
            left_vel = -left_vel
        if self.invert_right_wheel:
            right_pos = -right_pos
            right_vel = -right_vel

        orientation = self.imu_msg.orientation
        _, pitch, yaw = quaternion_to_euler(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

        pitch *= self.pitch_sign
        pitch_rate = self.pitch_sign * self.imu_msg.angular_velocity.y
        yaw_rate = self.imu_msg.angular_velocity.z

        theta = 0.5 * (left_pos + right_pos)
        theta_dot = 0.5 * (left_vel + right_vel)
        phi = yaw
        phi_dot = yaw_rate

        theta_dot_ref = float(self.get_parameter('theta_dot_ref').value)
        yaw_reference_mode = str(self.get_parameter('yaw_reference_mode').value)
        phi_dot_ref_target = float(self.get_parameter('phi_dot_ref').value)
        phi_ref_delta_request = float(self.get_parameter('phi_ref_delta_request').value)
        phi_ref_delta_request_id = int(self.get_parameter('phi_ref_delta_request_id').value)

        if phi_ref_delta_request_id != self.last_phi_ref_delta_request_id:
            self.phi_ref = wrap_to_pi(phi + phi_ref_delta_request)
            self.last_phi_ref_delta_request_id = phi_ref_delta_request_id
            self.get_logger().info(
                f'Yaw angle target set: phi_ref={self.phi_ref:.3f} rad, '
                f'delta={phi_ref_delta_request:.3f} rad'
            )

        if yaw_reference_mode == 'angle':
            phi_error_for_ref = wrap_to_pi(self.phi_ref - phi)
            phi_dot_ref_target = self.phi_ref_kp * phi_error_for_ref
            phi_dot_ref_target = float(
                np.clip(phi_dot_ref_target, -self.phi_dot_ref_limit, self.phi_dot_ref_limit)
            )
            if abs(phi_error_for_ref) < 0.01:
                phi_dot_ref_target = 0.0

        self.filtered_phi_dot_ref = self.limit_reference_slew_rate(
            phi_dot_ref_target,
            self.filtered_phi_dot_ref,
            self.phi_dot_ref_slew_rate,
            dt,
        )
        phi_dot_ref = self.filtered_phi_dot_ref
        reset_reference_request = bool(self.get_parameter('reset_reference_request').value)
        omega_just_stopped = (
            self.reset_yaw_on_omega_stop
            and abs(self.previous_phi_dot_ref) > self.omega_stop_threshold
            and abs(phi_dot_ref_target) <= self.omega_stop_threshold
        )

        if omega_just_stopped:
            self.phi_ref = phi
            self.get_logger().info(
                f'Omega stopped. Holding current yaw phi_ref={self.phi_ref:.3f}'
            )

        if not self.references_initialized or reset_reference_request:
            if self.hold_initial_position:
                self.theta_ref = theta
            if self.hold_initial_yaw:
                self.phi_ref = phi
                self.filtered_phi_dot_ref = phi_dot_ref_target
                phi_dot_ref = self.filtered_phi_dot_ref
            self.references_initialized = True
            if reset_reference_request:
                self.set_parameters([
                    Parameter('reset_reference_request', Parameter.Type.BOOL, False)
                ])
            self.get_logger().info(
                f'Stationary references initialized: theta_ref={self.theta_ref:.3f}, '
                f'phi_ref={self.phi_ref:.3f}, theta_dot_ref={theta_dot_ref:.3f}, '
                f'phi_dot_ref={phi_dot_ref:.3f}'
            )

        self.theta_ref += theta_dot_ref * dt
        if yaw_reference_mode != 'angle':
            self.phi_ref = wrap_to_pi(self.phi_ref + phi_dot_ref * dt)
        self.previous_phi_dot_ref = phi_dot_ref

        x_state = np.array([theta, theta_dot, pitch, pitch_rate, phi, phi_dot])
        x_ref = np.array([self.theta_ref, theta_dot_ref, 0.0, 0.0, self.phi_ref, phi_dot_ref])
        error = x_state - x_ref
        error[4] = wrap_to_pi(error[4])

        if abs(pitch) > self.max_tilt_rad:
            self.publish_torque(0.0, 0.0)
            self.get_logger().warn(
                f'Tilt {pitch:.3f} rad is above safety limit. Publishing zero torque.',
                throttle_duration_sec=1.0,
            )
            return

        torque = -self.torque_sign * (self.k_gain @ error)
        yaw_rate_error = phi_dot - phi_dot_ref
        yaw_brake = -self.yaw_torque_sign * self.yaw_rate_brake_gain * yaw_rate_error
        if self.yaw_rate_brake_limit > 0.0:
            yaw_brake = float(
                np.clip(yaw_brake, -self.yaw_rate_brake_limit, self.yaw_rate_brake_limit)
            )
        yaw_feedforward = 0.0
        if abs(phi_dot_ref) > self.yaw_torque_feedforward_deadband:
            speed_fraction = min(abs(phi_dot_ref) / max(self.phi_dot_ref_limit, 1e-6), 1.0)
            feedforward_magnitude = self.yaw_torque_feedforward * (0.5 + 0.5 * speed_fraction)
            yaw_feedforward = (
                self.yaw_torque_sign
                * feedforward_magnitude
                * math.copysign(1.0, phi_dot_ref)
            )
        yaw_torque = yaw_brake + yaw_feedforward
        if self.yaw_torque_max > 0.0:
            yaw_torque = float(np.clip(yaw_torque, -self.yaw_torque_max, self.yaw_torque_max))
        torque += np.array([yaw_torque, -yaw_torque])

        now_seconds = now.nanoseconds * 1e-9
        if (
            abs(phi_dot_ref) > self.omega_stop_threshold
            and now_seconds - self.last_yaw_debug_time > 1.0
        ):
            self.last_yaw_debug_time = now_seconds
            self.get_logger().info(
                f'yaw mode={yaw_reference_mode}, phi={phi:.3f}, phi_ref={self.phi_ref:.3f}, '
                f'phi_dot={phi_dot:.3f}, phi_dot_ref={phi_dot_ref:.3f}, '
                f'yaw_torque={yaw_torque:.3f}'
            )

        torque = np.clip(torque, -self.torque_limit, self.torque_limit)
        torque[0] = self.limit_wheel_speed(torque[0], left_vel)
        torque[1] = self.limit_wheel_speed(torque[1], right_vel)
        torque = self.limit_torque_slew_rate(torque, dt)
        torque = np.clip(torque, -self.torque_limit, self.torque_limit)
        self.last_torque = torque
        self.publish_torque(torque[0], torque[1])


def main():
    rclpy.init()
    node = LqrController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_torque(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
