# 文件名: downsample_pcd_simple.py
import open3d as o3d
import os

# ================== 配置区 ==================
PCD_DIR = "/home/mao/PCD"          # 固定目录
DEFAULT_VOXEL_SIZE = 0.05          # 默认体素大小（5cm）
# ========================================================

def main():
    print("          PCD 点云降采样工具")
    print(f"         固定路径：{PCD_DIR}")

    # 1. 输入文件名
    while True:
        input_name = input(f"\n请输入要降采样的文件名: ").strip()
        if not input_name.lower().endswith('.pcd'):
            input_name += '.pcd'
        input_path = os.path.join(PCD_DIR, input_name)

        if os.path.exists(input_path):
            print(f"找到文件: {input_path}")
            break
        else:
            print("文件不存在！请检查文件名是否正确（区分大小写）")

    # 2. 读取点云
    print("正在读取点云...")
    pcd = o3d.io.read_point_cloud(input_path)
    print(f"原始点数: {len(pcd.points):,}")

    # 3. 体素大小
    voxel_input = input(f"\n体素大小（回车使用默认 {DEFAULT_VOXEL_SIZE}m）: ").strip()
    try:
        voxel_size = float(voxel_input) if voxel_input else DEFAULT_VOXEL_SIZE
    except:
        voxel_size = DEFAULT_VOXEL_SIZE
        print(f"输入无效，使用默认值: {voxel_size}m")

    # 4. 降采样
    print(f"正在降采样（voxel_size = {voxel_size}m）...")
    down_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    print(f"降采样后点数: {len(down_pcd.points):,}  ↓ {100*(1 - len(down_pcd.points)/len(pcd.points)):.1f}%")

    # 5. 输出文件名
    while True:
        output_name = input(f"\n请输入输出文件名（回车自动生成: down_{input_name}）: ").strip()
        if not output_name:
            output_name = f"down_{input_name}"
        if not output_name.lower().endswith('.pcd'):
            output_name += '.pcd'
        
        output_path = os.path.join(PCD_DIR, output_name)
        if os.path.exists(output_path):
            overwrite = input(f"文件已存在！是否覆盖？(y/N): ").strip().lower()
            if overwrite in ['y', 'yes', 'Y']:
                break
        else:
            break

    # 6. 保存
    success = o3d.io.write_point_cloud(output_path, down_pcd)
    if success:
        print(f"\n成功保存！")
        print(f"路径: {output_path}")
    else:
        print("\n保存失败！")

    # 7. 是否可视化
    view = input("\n是否可视化对比？(y/N): ").strip().lower()
    if view in ['y', 'yes']:
        pcd.paint_uniform_color([1, 0.7, 0.7])      # 原图浅红
        down_pcd.paint_uniform_color([0.3, 1, 0.3])  # 降采样浅绿
        o3d.visualization.draw_geometries([
            pcd.translate((-1.2, 0, 0)),
            down_pcd.translate((1.2, 0, 0))
        ], window_name="左: 原始   右: 降采样后")

if __name__ == "__main__":
    main()