#!/usr/bin/python3
import math
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient


class LqrRefGui(Node):
    def __init__(self):
        super().__init__('lqr_ref_gui')
        self.declare_parameter('target_node', '/self_balance_lqr_controller')
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('linear_limit', 0.5)
        self.declare_parameter('omega_limit', 1.0)
        self.declare_parameter('yaw_angle_limit_deg', 180.0)

        self.target_node = self.get_parameter('target_node').value
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.linear_limit = float(self.get_parameter('linear_limit').value)
        self.omega_limit = float(self.get_parameter('omega_limit').value)
        self.yaw_angle_limit_deg = float(self.get_parameter('yaw_angle_limit_deg').value)
        self.client = AsyncParameterClient(self, self.target_node)
        self.yaw_request_id = 0

    def set_parameters(self, parameters):
        if not self.client.wait_for_services(timeout_sec=1.0):
            raise RuntimeError(f'Parameter service is not available for {self.target_node}')
        return self.client.set_parameters(parameters)

    def set_refs(self, linear_velocity, omega):
        theta_dot_ref = linear_velocity / self.wheel_radius
        return self.set_parameters([
            Parameter('yaw_reference_mode', Parameter.Type.STRING, 'rate'),
            Parameter('theta_dot_ref', Parameter.Type.DOUBLE, float(theta_dot_ref)),
            Parameter('phi_dot_ref', Parameter.Type.DOUBLE, float(omega)),
        ])

    def set_yaw_delta(self, yaw_delta_deg):
        self.yaw_request_id += 1
        yaw_delta_rad = math.radians(yaw_delta_deg)
        return self.set_parameters([
            Parameter('yaw_reference_mode', Parameter.Type.STRING, 'angle'),
            Parameter('theta_dot_ref', Parameter.Type.DOUBLE, 0.0),
            Parameter('phi_dot_ref', Parameter.Type.DOUBLE, 0.0),
            Parameter('phi_ref_delta_request', Parameter.Type.DOUBLE, float(yaw_delta_rad)),
            Parameter('phi_ref_delta_request_id', Parameter.Type.INTEGER, self.yaw_request_id),
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
        self.root.geometry('460x390')
        self.root.resizable(False, False)

        self.linear = tk.DoubleVar(value=0.0)
        self.omega = tk.DoubleVar(value=0.0)
        self.yaw_angle = tk.DoubleVar(value=0.0)
        self.status = tk.StringVar(value=f'Target: {node.target_node}')

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

        ttk.Label(self.root, text='Yaw rate omega_ref (rad/s)').grid(row=2, column=0, sticky='w', **pad)
        self.omega_value = ttk.Label(self.root, text='0.000')
        self.omega_value.grid(row=2, column=1, sticky='e', **pad)
        omega_slider = ttk.Scale(
            self.root,
            from_=-node.omega_limit,
            to=node.omega_limit,
            orient='horizontal',
            variable=self.omega,
            command=lambda _: self.update_labels(),
        )
        omega_slider.grid(row=3, column=0, columnspan=2, sticky='ew', **pad)

        ttk.Label(self.root, text='Yaw angle delta (deg)').grid(row=4, column=0, sticky='w', **pad)
        self.yaw_angle_value = ttk.Label(self.root, text='0.0')
        self.yaw_angle_value.grid(row=4, column=1, sticky='e', **pad)
        yaw_angle_slider = ttk.Scale(
            self.root,
            from_=-node.yaw_angle_limit_deg,
            to=node.yaw_angle_limit_deg,
            orient='horizontal',
            variable=self.yaw_angle,
            command=lambda _: self.update_labels(),
        )
        yaw_angle_slider.grid(row=5, column=0, columnspan=2, sticky='ew', **pad)

        rate_button_frame = ttk.Frame(self.root)
        rate_button_frame.grid(row=6, column=0, columnspan=2, sticky='ew', **pad)
        ttk.Button(rate_button_frame, text='Send v / Omega', command=self.send_refs).pack(
            side='left', expand=True, fill='x', padx=4
        )
        ttk.Button(rate_button_frame, text='Stop', command=self.stop).pack(
            side='left', expand=True, fill='x', padx=4
        )

        angle_button_frame = ttk.Frame(self.root)
        angle_button_frame.grid(row=7, column=0, columnspan=2, sticky='ew', **pad)
        ttk.Button(angle_button_frame, text='Rotate Angle', command=self.rotate_angle).pack(
            side='left', expand=True, fill='x', padx=4
        )
        ttk.Button(angle_button_frame, text='-90 deg', command=lambda: self.rotate_preset(-90.0)).pack(
            side='left', expand=True, fill='x', padx=4
        )
        ttk.Button(angle_button_frame, text='+90 deg', command=lambda: self.rotate_preset(90.0)).pack(
            side='left', expand=True, fill='x', padx=4
        )
        ttk.Button(angle_button_frame, text='Hold Current', command=self.hold_current).pack(
            side='left', expand=True, fill='x', padx=4
        )

        ttk.Label(self.root, textvariable=self.status).grid(row=8, column=0, columnspan=2, sticky='w', **pad)
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=0)

        self.update_labels()

    def update_labels(self):
        self.linear_value.configure(text=f'{self.linear.get():.3f}')
        self.omega_value.configure(text=f'{self.omega.get():.3f}')
        self.yaw_angle_value.configure(text=f'{self.yaw_angle.get():.1f}')

    def set_status_from_future(self, future, ok_message):
        try:
            results = future.result()
            if results and all(result.successful for result in results):
                self.status.set(ok_message)
            else:
                reason = results[0].reason if results else 'unknown reason'
                self.status.set(f'Parameter set failed: {reason}')
        except Exception as exc:
            self.status.set(f'Parameter set failed: {exc}')

    def send_refs(self):
        try:
            future = self.node.set_refs(self.linear.get(), self.omega.get())
        except Exception as exc:
            self.status.set(f'Parameter set failed: {exc}')
            return
        future.add_done_callback(
            lambda f: self.root.after(
                0,
                self.set_status_from_future,
                f,
                f'Sent v={self.linear.get():.3f} m/s, omega={self.omega.get():.3f} rad/s',
            )
        )

    def rotate_angle(self):
        self.linear.set(0.0)
        self.omega.set(0.0)
        self.update_labels()
        yaw_delta = self.yaw_angle.get()
        try:
            future = self.node.set_yaw_delta(yaw_delta)
        except Exception as exc:
            self.status.set(f'Parameter set failed: {exc}')
            return
        future.add_done_callback(
            lambda f: self.root.after(
                0,
                self.set_status_from_future,
                f,
                f'Rotating {yaw_delta:.1f} deg in place',
            )
        )

    def rotate_preset(self, yaw_delta):
        self.yaw_angle.set(yaw_delta)
        self.rotate_angle()

    def stop(self):
        self.linear.set(0.0)
        self.omega.set(0.0)
        self.update_labels()
        self.hold_current()

    def hold_current(self):
        self.linear.set(0.0)
        self.omega.set(0.0)
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
