import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tkinter as tk
from tkinter import ttk
import threading
import sys

class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')

        if len(sys.argv) > 1:
            topic_name = sys.argv[1]
        else:
            topic_name = '/low_level_joint_group_effort_controller/joint_trajectory'

        self.trajectory_publisher_ = self.create_publisher(JointTrajectory, topic_name, 10)
        self.trajectory = [
            0.0,
            1.0143535137176514,
            -2.0287070274353027,

            0.0,
            1.0143535137176514,
            -2.0287070274353027,

            0.0,
            1.0143535137176514,
            -2.0287070274353027,
            
            0.0,
            1.0143535137176514,
            -2.0287070274353027,
        ]
        self.trajectory_joint_names = [
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint'
        ]

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

    def publish_trajectory(self, positions):
        msg = JointTrajectory()
        msg.joint_names = self.trajectory_joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 16666666
        msg.points.append(point)
        self.trajectory_publisher_.publish(msg)
        self.get_logger().info(f'Trajectory message published: {positions}')
    
    def publish_velocity(self, velocities):
        msg = Float64MultiArray()
        msg.data = velocities
        self.velocity_publisher_.publish(msg)
        self.get_logger().info(f'Velocity message published: {velocities}')

def main(args=None):

    rclpy.init(args=args)
    node = JointPublisher()

    def on_slider_change(idx, var):
        if idx < len(node.trajectory):
            positions[idx] = var.get()
            node.publish_trajectory(positions)
        else:
            velocities[idx - len(node.trajectory)] = var.get()
            node.publish_velocity(velocities)

    positions = node.trajectory.copy()
    velocities = node.velocity.copy()

    def ros_spin():
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

    spin_thread = threading.Thread(target=ros_spin, daemon=True)
    spin_thread.start()

    root = tk.Tk()
    root.title("Trajectory Controller")

    sliders = []
    slider_vars = []

    for i, name in enumerate(node.trajectory_joint_names + node.velocity_joint_names):
        frame = ttk.Frame(root)
        frame.pack(fill='x', padx=5, pady=2)
        if i == 0:
            label = ttk.Label(frame, text="\tTrajectory Joints")
            label.pack(side='left')
            frame = ttk.Frame(root)
            frame.pack(fill='x', padx=5, pady=2)
        elif i == len(node.trajectory_joint_names):
            label = ttk.Label(frame, text="\tVelocity Joints")
            label.pack(side='left')
            frame = ttk.Frame(root)
            frame.pack(fill='x', padx=5, pady=2)
        label = ttk.Label(frame, text=f"{i+1}. {name}")
        label.pack(side='left')
        var = tk.DoubleVar(value=(positions+velocities)[i])

        # Définir les bornes des sliders selon l'index
        if i in [0, 3, 6, 9]:  # Hip
            slider = ttk.Scale(frame, from_=-1.0472, to=1.0472, orient='horizontal', variable=var,
                       command=lambda val, idx=i, v=var: on_slider_change(idx, v))
        elif i in [1, 4, 7, 10]:  # Thigh
            slider = ttk.Scale(frame, from_=-1.5708, to=3.4907, orient='horizontal', variable=var,
                       command=lambda val, idx=i, v=var: on_slider_change(idx, v))
        elif i in [2, 5, 8, 11]:  # Calf
            slider = ttk.Scale(frame, from_=-2.7227, to=-0.83776, orient='horizontal', variable=var,
                       command=lambda val, idx=i, v=var: on_slider_change(idx, v))
        else:
            slider = ttk.Scale(frame, from_=-5.0, to=5.0, orient='horizontal', variable=var,
                       command=lambda val, idx=i, v=var: on_slider_change(idx, v))
            
        slider.pack(side='right', fill='none', expand=False, padx=5)
        slider.configure(length=100)  # Par exemple, 200 pixels de long
        frame.pack_propagate(False)
        frame.configure(width=230, height=30)
        sliders.append(slider)
        slider_vars.append(var)

    def on_close():
        node.destroy_node()
        rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()

if __name__ == '__main__':
    main()
