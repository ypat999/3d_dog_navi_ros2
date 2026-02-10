#!/usr/bin/env python3
"""
Gazebo Garden 迁移测试脚本
用于验证转换后的模型和世界文件
"""

import os
import subprocess
import time
import sys

def check_gazebo_garden_installation():
    """检查Gazebo Garden是否已安装"""
    try:
        result = subprocess.run(['gz', '--version'], capture_output=True, text=True)
        if result.returncode == 0:
            print(f"✓ Gazebo Garden已安装: {result.stdout.strip()}")
            return True
        else:
            print("✗ Gazebo Garden未安装或未找到")
            return False
    except FileNotFoundError:
        print("✗ Gazebo Garden未安装")
        return False

def check_ros_gz_packages():
    """检查ROS 2 Gazebo集成包"""
    packages = ['ros_gz_sim', 'ros_gz_bridge']
    missing_packages = []
    
    for pkg in packages:
        try:
            result = subprocess.run(['ros2', 'pkg', 'list'], capture_output=True, text=True)
            if pkg in result.stdout:
                print(f"✓ {pkg} 包已安装")
            else:
                print(f"✗ {pkg} 包未安装")
                missing_packages.append(pkg)
        except Exception as e:
            print(f"✗ 检查包 {pkg} 时出错: {e}")
            missing_packages.append(pkg)
    
    return len(missing_packages) == 0

def validate_world_file():
    """验证世界文件格式"""
    world_file = os.path.join(os.path.dirname(__file__), 'worlds/Building.world')
    
    if os.path.exists(world_file):
        with open(world_file, 'r') as f:
            content = f.read()
            if 'sdf version="1.9"' in content:
                print("✓ 世界文件使用SDFormat 1.9")
                return True
            else:
                print("✗ 世界文件版本不正确")
                return False
    else:
        print("✗ 世界文件不存在")
        return False

def validate_model_files():
    """验证模型文件"""
    model_dir = os.path.join(os.path.dirname(__file__), 'models/go2w')
    required_files = ['model.config', 'go2w.sdf']
    
    if os.path.exists(model_dir):
        for file in required_files:
            file_path = os.path.join(model_dir, file)
            if os.path.exists(file_path):
                print(f"✓ {file} 文件存在")
            else:
                print(f"✗ {file} 文件不存在")
                return False
        return True
    else:
        print("✗ 模型目录不存在")
        return False

def test_gazebo_simulation():
    """测试Gazebo仿真"""
    world_file = os.path.join(os.path.dirname(__file__), 'worlds/Building.world')
    
    print("\n启动Gazebo Garden仿真测试...")
    
    try:
        # 启动Gazebo Garden（短暂运行以测试）
        process = subprocess.Popen(
            ['gz', 'sim', '-v', '1', '-r', world_file],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        # 等待几秒钟让仿真启动
        time.sleep(5)
        
        # 检查进程是否仍在运行
        if process.poll() is None:
            print("✓ Gazebo Garden仿真成功启动")
            process.terminate()
            process.wait(timeout=5)
            return True
        else:
            stdout, stderr = process.communicate()
            print(f"✗ Gazebo Garden启动失败: {stderr}")
            return False
            
    except Exception as e:
        print(f"✗ 测试过程中出错: {e}")
        return False

def main():
    print("=== Gazebo Garden 迁移测试 ===\n")
    
    tests = [
        ("检查Gazebo Garden安装", check_gazebo_garden_installation),
        ("检查ROS 2集成包", check_ros_gz_packages),
        ("验证世界文件", validate_world_file),
        ("验证模型文件", validate_model_files),
        ("测试仿真启动", test_gazebo_simulation)
    ]
    
    all_passed = True
    
    for test_name, test_func in tests:
        print(f"\n{test_name}:")
        if test_func():
            print(f"  ✓ {test_name} 通过")
        else:
            print(f"  ✗ {test_name} 失败")
            all_passed = False
    
    print("\n" + "="*50)
    if all_passed:
        print("✓ 所有测试通过！迁移成功完成。")
        print("\n下一步操作:")
        print("1. 使用 'ros2 launch gazebo_garden_migration gazebo_garden.launch.py' 启动完整仿真")
        print("2. 检查机器狗模型是否正确加载")
        print("3. 验证物理仿真和传感器数据")
    else:
        print("✗ 部分测试失败，请检查上述错误信息。")
        sys.exit(1)

if __name__ == "__main__":
    main()