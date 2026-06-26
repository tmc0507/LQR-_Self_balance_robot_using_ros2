#!/usr/bin/python3
import math
import threading
import tkinter as tk
from collections import deque
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
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


class LqrRefGui(Node):
    def __init__(self):
        super().__init__('lqr_ref_gui')
        self.declare_parameter('target_node', '/self_balance_lqr_controller')
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('linear_limit', 0.5)
        self.declare_parameter('phi_angle_limit_deg', 90.0)
        self.declare_parameter('torque_plot_limit', 0.0)
        self.declare_parameter('command_topic', '/wheel_effort_controller/commands')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('left_joint', 'left_wheeL_joint')
        self.declare_parameter('right_joint', 'right_wheel_joint')

        self.target_node = self.get_parameter('target_node').value
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.linear_limit = float(self.get_parameter('linear_limit').value)
        self.phi_angle_limit_deg = float(self.get_parameter('phi_angle_limit_deg').value)
        self.torque_plot_limit = float(self.get_parameter('torque_plot_limit').value)
        self.left_joint = self.get_parameter('left_joint').value
        self.right_joint = self.get_parameter('right_joint').value
        self.client = AsyncParameterClient(self, self.target_node)
        self.latest_state = [0.0] * 6
        self.latest_torque = [0.0, 0.0]
        self.latest_wheel_state = [0.0, 0.0]
        self.imu_msg = None
        self.joint_msg = None

        self.create_subscription(
            Float64MultiArray,
            self.get_parameter('command_topic').value,
            self.command_callback,
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

    def command_callback(self, msg):
        if len(msg.data) >= 2:
            self.latest_torque = [float(msg.data[0]), float(msg.data[1])]

    def imu_callback(self, msg):
        self.imu_msg = msg
        self.update_state()

    def joint_state_callback(self, msg):
        self.joint_msg = msg
        self.update_state()

    def get_joint_state(self, joint_name):
        if self.joint_msg is None or joint_name not in self.joint_msg.name:
            return None, None

        index = self.joint_msg.name.index(joint_name)
        position = self.joint_msg.position[index] if index < len(self.joint_msg.position) else 0.0
        velocity = self.joint_msg.velocity[index] if index < len(self.joint_msg.velocity) else 0.0
        return position, velocity

    def update_state(self):
        if self.imu_msg is None or self.joint_msg is None:
            return

        left_pos, left_vel = self.get_joint_state(self.left_joint)
        right_pos, right_vel = self.get_joint_state(self.right_joint)
        if left_pos is None or right_pos is None:
            return

        orientation = self.imu_msg.orientation
        _, pitch, yaw = quaternion_to_euler(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

        theta = 0.5 * (left_pos + right_pos)
        theta_dot = 0.5 * (left_vel + right_vel)
        pitch_rate = self.imu_msg.angular_velocity.y
        phi_dot = self.imu_msg.angular_velocity.z
        self.latest_state = [theta, theta_dot, pitch, pitch_rate, yaw, phi_dot]
        self.latest_wheel_state = [left_vel, right_vel]

    def set_parameters(self, parameters):
        if not self.client.wait_for_services(timeout_sec=1.0):
            raise RuntimeError(f'Parameter service is not available for {self.target_node}')
        return self.client.set_parameters(parameters)

    def set_refs(self, linear_velocity, phi_delta_deg):
        theta_dot_ref = linear_velocity / self.wheel_radius
        phi_delta_rad = math.radians(phi_delta_deg)
        request_id = int(self.get_clock().now().nanoseconds)
        return self.set_parameters([
            Parameter('yaw_reference_mode', Parameter.Type.STRING, 'angle'),
            Parameter('theta_dot_ref', Parameter.Type.DOUBLE, float(theta_dot_ref)),
            Parameter('phi_dot_ref', Parameter.Type.DOUBLE, 0.0),
            Parameter('phi_ref_delta_request', Parameter.Type.DOUBLE, float(phi_delta_rad)),
            Parameter('phi_ref_delta_request_id', Parameter.Type.INTEGER, request_id),
        ])

    def hold_current_pose(self):
        return self.set_parameters([
            Parameter('yaw_reference_mode', Parameter.Type.STRING, 'rate'),
            Parameter('theta_dot_ref', Parameter.Type.DOUBLE, 0.0),
            Parameter('phi_dot_ref', Parameter.Type.DOUBLE, 0.0),
            Parameter('reset_reference_request', Parameter.Type.BOOL, True),
        ])


class GuiApp:
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title('Self Balance LQR Reference')
        self.root.geometry('1100x820')
        self.root.minsize(900, 680)
        self.root.resizable(True, True)

        self.linear = tk.DoubleVar(value=0.0)
        self.phi_angle = tk.DoubleVar(value=0.0)
        self.status = tk.StringVar(value=f'Target: {node.target_node}')
        self.torque_history = deque(maxlen=220)
        self.state_values = {}

        pad = {'padx': 12, 'pady': 6}

        ttk.Label(self.root, text='Linear velocity v_ref (m/s)').grid(row=0, column=0, sticky='w', **pad)
        self.linear_value = ttk.Label(self.root, text='0.000')
        self.linear_value.grid(row=0, column=1, sticky='e', **pad)
        linear_slider = ttk.Scale(
            self.root,
            from_=-node.linear_limit,
            to=node.linear_limit,
            orient='horizontal',
            variable=self.linear,
            command=lambda _: self.update_labels(),
        )
        linear_slider.grid(row=1, column=0, columnspan=2, sticky='ew', **pad)

        ttk.Label(self.root, text='Yaw angle phi_ref delta (deg)').grid(row=2, column=0, sticky='w', **pad)
        self.phi_angle_value = ttk.Label(self.root, text='0.0')
        self.phi_angle_value.grid(row=2, column=1, sticky='e', **pad)
        phi_angle_slider = ttk.Scale(
            self.root,
            from_=-node.phi_angle_limit_deg,
            to=node.phi_angle_limit_deg,
            orient='horizontal',
            variable=self.phi_angle,
            command=lambda _: self.update_labels(),
        )
        phi_angle_slider.grid(row=3, column=0, columnspan=2, sticky='ew', **pad)

        rate_button_frame = ttk.Frame(self.root)
        rate_button_frame.grid(row=4, column=0, columnspan=2, sticky='ew', **pad)
        ttk.Button(rate_button_frame, text='Send v / Phi', command=self.send_refs).pack(
            side='left', expand=True, fill='x', padx=4
        )
        ttk.Button(rate_button_frame, text='Stop', command=self.stop).pack(
            side='left', expand=True, fill='x', padx=4
        )
        ttk.Button(rate_button_frame, text='Hold Current', command=self.hold_current).pack(
            side='left', expand=True, fill='x', padx=4
        )

        ttk.Label(self.root, textvariable=self.status).grid(row=5, column=0, columnspan=2, sticky='w', **pad)
        self.create_monitor_widgets(pad)
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=0)
        self.root.rowconfigure(6, weight=1)

        self.update_labels()
        self.refresh_monitor()

    def create_monitor_widgets(self, pad):
        plot_frame = ttk.LabelFrame(self.root, text='Motor torque command (N.m)')
        plot_frame.grid(row=6, column=0, columnspan=2, sticky='nsew', **pad)
        plot_frame.rowconfigure(0, weight=1)
        plot_frame.columnconfigure(0, weight=1)
        self.plot_canvas = tk.Canvas(plot_frame, width=1040, height=360, bg='white', highlightthickness=1)
        self.plot_canvas.pack(fill='both', expand=True, padx=8, pady=8)

        legend_frame = ttk.Frame(plot_frame)
        legend_frame.pack(fill='x', padx=8, pady=(0, 6))
        ttk.Label(legend_frame, text='left motor', foreground='#2563eb').pack(side='left', padx=(0, 16))
        ttk.Label(legend_frame, text='right motor', foreground='#dc2626').pack(side='left')

        state_frame = ttk.LabelFrame(self.root, text='LQR state vector')
        state_frame.grid(row=7, column=0, columnspan=2, sticky='ew', **pad)

        headings = [('State', 18), ('Value', 16), ('Unit', 12)]
        for column, (heading, width) in enumerate(headings):
            label = ttk.Label(state_frame, text=heading)
            label.grid(row=0, column=column, sticky='w', padx=8, pady=4)
            state_frame.columnconfigure(column, minsize=width * 8)

        rows = [
            ('theta', 'rad'),
            ('theta_dot', 'rad/s'),
            ('pitch', 'rad'),
            ('pitch_rate', 'rad/s'),
            ('phi', 'rad'),
            ('phi_dot', 'rad/s'),
            ('left_wheel_dot', 'rad/s'),
            ('right_wheel_dot', 'rad/s'),
        ]
        for row_index, (name, unit) in enumerate(rows, start=1):
            ttk.Label(state_frame, text=name).grid(row=row_index, column=0, sticky='w', padx=8, pady=2)
            value_label = ttk.Label(state_frame, text='0.0000')
            value_label.grid(row=row_index, column=1, sticky='w', padx=8, pady=2)
            ttk.Label(state_frame, text=unit).grid(row=row_index, column=2, sticky='w', padx=8, pady=2)
            self.state_values[name] = value_label

    def update_labels(self):
        self.linear_value.configure(text=f'{self.linear.get():.3f}')
        self.phi_angle_value.configure(text=f'{self.phi_angle.get():.1f}')

    def refresh_monitor(self):
        self.torque_history.append(tuple(self.node.latest_torque))
        self.update_state_table()
        self.draw_torque_plot()
        self.root.after(100, self.refresh_monitor)

    def update_state_table(self):
        names = ['theta', 'theta_dot', 'pitch', 'pitch_rate', 'phi', 'phi_dot']
        for name, value in zip(names, self.node.latest_state):
            self.state_values[name].configure(text=f'{value:.4f}')
        wheel_names = ['left_wheel_dot', 'right_wheel_dot']
        for name, value in zip(wheel_names, self.node.latest_wheel_state):
            self.state_values[name].configure(text=f'{value:.4f}')

    def draw_torque_plot(self):
        canvas = self.plot_canvas
        width = max(canvas.winfo_width(), int(canvas['width']))
        height = max(canvas.winfo_height(), int(canvas['height']))
        margin = 24
        canvas.delete('all')

        canvas.create_line(margin, height / 2, width - margin, height / 2, fill='#d1d5db')
        canvas.create_rectangle(margin, margin, width - margin, height - margin, outline='#e5e7eb')

        if len(self.torque_history) < 2:
            return

        max_torque = max(abs(value) for sample in self.torque_history for value in sample)
        max_abs = self.node.torque_plot_limit if self.node.torque_plot_limit > 0.0 else max_torque
        max_abs = max(0.1, max_abs)
        canvas.create_text(margin + 4, margin - 10, anchor='w', text=f'+/-{max_abs:.2f} N.m')

        def point(index, value):
            x_span = width - 2 * margin
            y_span = height - 2 * margin
            x = margin + x_span * index / max(len(self.torque_history) - 1, 1)
            y = height / 2 - (value / max_abs) * (y_span / 2)
            return x, y

        for motor_index, color in enumerate(('#2563eb', '#dc2626')):
            points = []
            for index, sample in enumerate(self.torque_history):
                points.extend(point(index, sample[motor_index]))
            if len(points) >= 4:
                canvas.create_line(*points, fill=color, width=2)

    def set_status_from_future(self, future, ok_message):
        try:
            response = future.result()
            results = response.results if hasattr(response, 'results') else response
            if results and all(result.successful for result in results):
                self.status.set(ok_message)
            else:
                reason = results[0].reason if results else 'unknown reason'
                self.status.set(f'Parameter set failed: {reason}')
        except Exception as exc:
            self.status.set(f'Parameter set failed: {exc}')

    def send_refs(self):
        try:
            future = self.node.set_refs(self.linear.get(), self.phi_angle.get())
        except Exception as exc:
            self.status.set(f'Parameter set failed: {exc}')
            return
        future.add_done_callback(
            lambda f: self.root.after(
                0,
                self.set_status_from_future,
                f,
                f'Sent v={self.linear.get():.3f} m/s, phi_delta={self.phi_angle.get():.1f} deg',
            )
        )

    def stop(self):
        self.linear.set(0.0)
        self.phi_angle.set(0.0)
        self.update_labels()
        self.hold_current()

    def hold_current(self):
        self.linear.set(0.0)
        self.phi_angle.set(0.0)
        self.update_labels()
        try:
            future = self.node.hold_current_pose()
        except Exception as exc:
            self.status.set(f'Parameter set failed: {exc}')
            return
        future.add_done_callback(
            lambda f: self.root.after(
                0,
                self.set_status_from_future,
                f,
                'Stopped and reset references to current pose',
            )
        )

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = LqrRefGui()
    executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    executor_thread.start()

    app = GuiApp(node)
    try:
        app.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
