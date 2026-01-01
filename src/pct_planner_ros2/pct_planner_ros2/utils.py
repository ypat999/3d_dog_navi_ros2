import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


def traj2ros(traj):
    path_msg = Path()
    path_msg.header.frame_id = "map"
    # 时间戳将在发布时设置

    for waypoint in traj:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        # 确保数据类型为float
        pose_stamped.pose.position.x = float(waypoint[0])
        pose_stamped.pose.position.y = float(waypoint[1])
        pose_stamped.pose.position.z = float(waypoint[2])
        pose_stamped.pose.orientation.w = 1.0
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        path_msg.poses.append(pose_stamped)

    return path_msg