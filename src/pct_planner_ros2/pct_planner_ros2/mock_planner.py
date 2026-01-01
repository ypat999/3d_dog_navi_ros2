import numpy as np


class MockAStar:
    def get_result_matrix(self):
        return np.array([])  # 模拟返回空路径


class MockTrajOptimizer:
    def get_opt_init_value(self):
        return np.array([])
    
    def get_opt_init_layer(self):
        return np.array([])
    
    def get_result_matrix(self):
        return np.array([])
    
    def get_layers(self):
        return np.array([])
    
    def get_heights(self):
        return np.array([])


class MockElePlanner:
    def __init__(self, max_heading_rate=10.0, use_quintic=True):
        self.max_heading_rate = max_heading_rate
        self.use_quintic = use_quintic
        self.path_finder = MockAStar()
        self.traj_optimizer = MockTrajOptimizer()
    
    def init_map(self, *args):
        print("Initializing map with mock data")
    
    def plan(self, start_idx, end_idx, visualize=False):
        print(f"Planning from {start_idx} to {end_idx}")
    
    def get_path_finder(self):
        return self.path_finder
    
    def get_trajectory_optimizer(self):
        return self.traj_optimizer
    
    def get_trajectory_optimizer_wnoj(self):
        return self.traj_optimizer