class ConfigPlanner():
    use_quintic = True
    max_heading_rate = 10.0  # 确保是浮点数


class ConfigWrapper():
    tomo_dir = '../rsc/tomogram/'  # 更新路径以适应ROS2包结构


class Config():
    def __init__(self):
        self.planner = ConfigPlanner()
        self.wrapper = ConfigWrapper()