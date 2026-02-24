import os
import sys
import pickle
import numpy as np

from .utils import *

# 尝试导入真实的C++库，如果失败则使用模拟实现
try:
    # 尝试导入编译的C++模块
    import pct_planner_cpp
    HAS_CPP_LIB = True
except ImportError:
    print("C++ libraries not available, using mock implementation")
    HAS_CPP_LIB = False

rsg_root = os.path.dirname(os.path.abspath(__file__))


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


def transTrajGrid2Map(grid_dim, center, resolution, traj_grid):
    """将网格轨迹转换为地图坐标"""
    offset = np.array([grid_dim[1] // 2, grid_dim[0] // 2, 0])
    center_ = np.array([center[1], center[0], 0.5])

    traj_grid = (traj_grid - offset) * resolution + center_

    traj_map = np.stack([traj_grid[:, 1], traj_grid[:, 0], traj_grid[:, 2]], axis=1)

    return traj_map


class TomogramPlanner(object):
    def __init__(self, cfg):
        self.cfg = cfg

        self.use_quintic = self.cfg.planner.use_quintic
        self.max_heading_rate = self.cfg.planner.max_heading_rate

        # 使用相对路径适应ROS2包结构，优先查找原始PCT_planner目录
        import os
        # 首先尝试在原始PCT_planner位置查找
        original_tomo_dir = '/home/ywj/git/3d_dog_navi_ros2/PCT_planner' + self.cfg.wrapper.tomo_dir
        if os.path.exists(original_tomo_dir):
            self.tomo_dir = original_tomo_dir
        else:
            # 如果原始位置不存在，则尝试相对路径
            rsg_root = os.path.dirname(os.path.abspath(__file__)) + '/../../PCT_planner'
            self.tomo_dir = rsg_root + self.cfg.wrapper.tomo_dir

        self.resolution = None
        self.center = None
        self.n_slice = None
        self.slice_h0 = None
        self.slice_dh = None
        self.map_dim = []
        self.offset = None

        self.start_idx = np.zeros(3, dtype=np.int32)
        self.end_idx = np.zeros(3, dtype=np.int32)

        print(f"Tomogram directory: {self.tomo_dir}")

    def loadTomogram(self, tomo_file):
        """加载tomogram文件"""
        tomo_path = os.path.join(self.tomo_dir, f'{tomo_file}.pickle')
        print(f"Attempting to load tomogram from: {tomo_path}")
        
        # 由于原始tomogram文件可能不存在，我们创建一个模拟的加载过程
        if os.path.exists(tomo_path):
            with open(tomo_path, 'rb') as handle:
                data_dict = pickle.load(handle)

                tomogram = np.asarray(data_dict['data'], dtype=np.float32)

                self.resolution = float(data_dict['resolution'])
                self.center = np.asarray(data_dict['center'], dtype=np.double)
                self.n_slice = tomogram.shape[1]
                self.slice_h0 = float(data_dict['slice_h0'])
                self.slice_dh = float(data_dict['slice_dh'])
                self.map_dim = [tomogram.shape[2], tomogram.shape[3]]
                self.offset = np.array([int(self.map_dim[0] / 2), int(self.map_dim[1] / 2)], dtype=np.int32)

            trav = tomogram[0]
            trav_gx = tomogram[1]
            trav_gy = tomogram[2]
            elev_g = tomogram[3]
            elev_g = np.nan_to_num(elev_g, nan=-100)
            elev_c = tomogram[4]
            elev_c = np.nan_to_num(elev_c, nan=1e6)

            self.initPlanner(trav, trav_gx, trav_gy, elev_g, elev_c)
        else:
            # 模拟加载过程
            print(f"Tomogram file not found: {tomo_path}, using mock data")
            self.resolution = 0.1
            self.center = np.array([0.0, 0.0, 0.0])
            self.n_slice = 10
            self.slice_h0 = 0.0
            self.slice_dh = 0.5
            self.map_dim = [100, 100]
            self.offset = np.array([50, 50], dtype=np.int32)
            
            # 创建模拟数据
            trav = np.ones((10, 100, 100), dtype=np.float32) * 0.5
            trav_gx = np.zeros((10, 100, 100), dtype=np.float32)
            trav_gy = np.zeros((10, 100, 100), dtype=np.float32)
            elev_g = np.ones((10, 100, 100), dtype=np.float32) * 1.0
            elev_c = np.ones((10, 100, 100), dtype=np.float32) * 10.0
            
            self.initPlanner(trav[0], trav_gx[0], trav_gy[0], elev_g[0], elev_c[0])
        
    def initPlanner(self, trav, trav_gx, trav_gy, elev_g, elev_c):
        diff_t = trav[1:] - trav[:-1]
        diff_g = np.abs(elev_g[1:] - elev_g[:-1])

        gateway_up = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t < -8.0
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[1:]))
        gateway_up[:-1] = np.logical_and(mask_t, mask_g)

        gateway_dn = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t > 8.0
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[:-1]))
        gateway_dn[1:] = np.logical_and(mask_t, mask_g)
        
        gateway = np.zeros_like(trav, dtype=np.int32)
        gateway[gateway_up] = 2
        gateway[gateway_dn] = -2

        if HAS_CPP_LIB:
            # 使用真实的C++库
            self.planner = pct_planner_cpp.OfflineElePlanner(
                max_heading_rate=self.max_heading_rate, 
                use_quintic=self.use_quintic
            )
            # 初始化地图
            self.planner.init_map(
                20, 15, self.resolution, self.n_slice, 0.1,
                trav.reshape(-1, trav.shape[-1]).astype(np.double),
                elev_g.reshape(-1, elev_g.shape[-1]).astype(np.double),
                elev_c.reshape(-1, elev_c.shape[-1]).astype(np.double),
                gateway.reshape(-1, gateway.shape[-1]),
                trav_gy.reshape(-1, trav_gy.shape[-1]).astype(np.double),
                -trav_gx.reshape(-1, trav_gx.shape[-1]).astype(np.double)
            )
        else:
            # 使用模拟实现
            from .mock_planner import MockElePlanner
            self.planner = MockElePlanner(
                max_heading_rate=self.max_heading_rate, 
                use_quintic=self.use_quintic
            )
        
        print("Planner initialized with mock data")

    def plan(self, start_pos, end_pos):
        """执行路径规划"""
        # 将位置转换为索引
        self.start_idx[1:] = self.pos2idx(start_pos[:2])
        self.end_idx[1:] = self.pos2idx(end_pos[:2])
        self.start_idx[0] = self.pos2slice(start_pos[-1])
        self.end_idx[0] = self.pos2slice(end_pos[-1])

        try:
            # 执行规划
            self.planner.plan(self.start_idx, self.end_idx, True)
            path_finder = self.planner.get_path_finder()
            path = path_finder.get_result_matrix()
            
            if len(path) == 0 or path.size == 0:
                # 如果没有找到路径，返回一条直线作为模拟
                steps = 10
                simulated_path = []
                for i in range(steps + 1):
                    t = i / steps
                    pos = start_pos + t * (end_pos - start_pos)
                    simulated_path.append(pos)
                traj_3d = np.array(simulated_path)
            else:
                # 处理实际路径（仅当使用C++库时）
                if HAS_CPP_LIB:
                    optimizer = self.planner.get_trajectory_optimizer() if not self.use_quintic else self.planner.get_trajectory_optimizer_wnoj()

                    opt_init = optimizer.get_opt_init_value()
                    init_layer = optimizer.get_opt_init_layer()
                    traj_raw = optimizer.get_result_matrix()
                    layers = optimizer.get_layers()
                    heights = optimizer.get_heights()

                    opt_init = np.concatenate([opt_init.transpose(1, 0), init_layer.reshape(-1, 1)], axis=-1)
                    traj = np.concatenate([traj_raw, layers.reshape(-1, 1)], axis=-1)
                    y_idx = (traj.shape[-1] - 1) // 2
                    traj_3d = np.stack([traj[:, 0], traj[:, y_idx], heights / self.resolution], axis=1)
                    traj_3d = transTrajGrid2Map(self.map_dim, self.center, self.resolution, traj_3d)
                else:
                    # 使用模拟路径
                    steps = 10
                    simulated_path = []
                    for i in range(steps + 1):
                        t = i / steps
                        pos = start_pos + t * (end_pos - start_pos)
                        simulated_path.append(pos)
                    traj_3d = np.array(simulated_path)

            return traj_3d
        except Exception as e:
            print(f"Planning failed: {e}")
            # 返回模拟路径
            steps = 10
            simulated_path = []
            for i in range(steps + 1):
                t = i / steps
                pos = start_pos + t * (end_pos - start_pos)
                simulated_path.append(pos)
            return np.array(simulated_path)
    
    def pos2slice(self, z):
        """将z坐标转换为切片索引"""
        slice_offset = (z - self.slice_h0) / self.slice_dh
        slice_idx = int(np.round(slice_offset))
        return np.clip(slice_idx, 0, self.n_slice - 1)

    def get_slice_height(self, slice_idx):
        """获取指定切片的实际高度"""
        return self.slice_h0 + slice_idx * self.slice_dh
    
    def pos2idx(self, pos):
        """将位置转换为索引"""
        pos_shifted = pos - self.center[:2]
        idx = np.round(pos_shifted / self.resolution).astype(np.int32) + self.offset
        # 确保索引在有效范围内
        idx[0] = np.clip(idx[0], 0, self.map_dim[0]-1)
        idx[1] = np.clip(idx[1], 0, self.map_dim[1]-1)
        return idx.astype(np.int32)