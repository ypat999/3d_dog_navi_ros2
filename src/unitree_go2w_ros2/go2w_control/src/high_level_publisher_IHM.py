import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import tkinter as tk
from tkinter import ttk
import threading

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.cmd_vel_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.velocity_publisher_ = self.create_publisher(Float64MultiArray, '/joint_group_velocity_controller/commands', 10)
        
        self.velocity = [
            0.0,
            0.0,
            0.0,
            0.0
        ]
        self.velocity_joint_names = [
            'FL_foot_joint',
            'FR_foot_joint',
            'RL_foot_joint',
            'RR_foot_joint'
        ]

    def publish_cmd_vel(self, linear_x, linear_y, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z

        self.cmd_vel_publisher_.publish(msg)
        self.get_logger().info(f'cmd_vel published: linear.x={linear_x}, linear.y={linear_y}, angular.z={angular_z}')

    def publish_velocity(self, velocities):
        msg = Float64MultiArray()
        msg.data = velocities
        self.velocity_publisher_.publish(msg)
        self.get_logger().info(f'Velocity message published: {velocities}')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()

    root = tk.Tk()
    root.title("High Level Controller")

    # Valeurs initiales
    linear_x = tk.DoubleVar(value=0.0)
    linear_y = tk.DoubleVar(value=0.0)
    angular_z = tk.DoubleVar(value=0.0)
    velocity_vars = [tk.DoubleVar(value=0.0) for _ in node.velocity_joint_names]

    def on_slider_change(_=None):
        node.publish_cmd_vel(linear_x.get(), linear_y.get(), angular_z.get())
        velocities = [v.get() for v in velocity_vars]
        node.publish_velocity(velocities)
    
    def on_button_click(var):
        var.set(0.0)
        on_slider_change()

    def ros_spin():
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

    spin_thread = threading.Thread(target=ros_spin, daemon=True)
    spin_thread.start()

    sliders_info = [
        ("Linear  X", linear_x, -0.4, 0.4),
        ("Linear  Y", linear_y, -0.3, 0.3),
        ("Angular Z", angular_z, -0.6, 0.6)
    ]

    for i, (label_text, var, minval, maxval) in enumerate(sliders_info):
        frame = ttk.Frame(root)
        frame.pack(fill='x', padx=5, pady=2)
        label = ttk.Label(frame, text=f"{label_text}")
        label.pack(side='left')
        label.configure(width=8)
        button = ttk.Button(frame, text="Reset", command=lambda v=var: on_button_click(v))
        button.pack(side='right', padx=5)
        slider = ttk.Scale(frame, from_=minval, to=maxval, orient='horizontal', variable=var,
                           command=on_slider_change)
        slider.pack(side='right', fill='x', expand=True, padx=5)
        slider.configure(length=150)
        frame.pack_propagate(False)
        frame.configure(width=400, height=40)

    # Ajout des sliders pour les velocity joints
    velocity_label = ttk.Label(root, text="Velocity Joints")
    velocity_label.pack(pady=(10, 0))
    for i, joint_name in enumerate(node.velocity_joint_names):
        frame = ttk.Frame(root)
        frame.pack(fill='x', padx=5, pady=2)
        label = ttk.Label(frame, text=f"{joint_name}")
        label.pack(side='left')
        button = ttk.Button(frame, text="Reset", command=lambda v=velocity_vars[i]: on_button_click(v))
        button.pack(side='right', padx=5)
        slider = ttk.Scale(frame, from_=-5.0, to=5.0, orient='horizontal', variable=velocity_vars[i],
                           command=on_slider_change)
        slider.pack(side='right', fill='x', expand=True, padx=5)
        slider.configure(length=150)
        frame.pack_propagate(False)
        frame.configure(width=400, height=40)

    def on_close():
        node.destroy_node()
        rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()

if __name__ == '__main__':
    main()
