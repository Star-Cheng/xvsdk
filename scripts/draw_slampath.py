import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def parse_slam_data(file_path):
    """
    从 txt 文件中解析 SLAM 数据，提取位置信息 (x, y, z)。
    """
    x_coords, y_coords, z_coords = [], [], []
    with open(file_path, 'r') as file:
        lines = file.readlines()
        for i in range(len(lines)):
            if "position:" in lines[i]:
                x = float(lines[i + 1].split(":")[1].strip())
                y = float(lines[i + 2].split(":")[1].strip())
                z = float(lines[i + 3].split(":")[1].strip())
                x_coords.append(x)
                y_coords.append(y)
                z_coords.append(z)
    return x_coords, y_coords, z_coords

def plot_3d_slam_path(file_path):
    """
    绘制三维 SLAM Path。
    """
    # 解析数据
    x_coords, y_coords, z_coords = parse_slam_data(file_path)

    # 创建 3D 图形
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # 绘制 SLAM Path
    ax.plot(x_coords, y_coords, z_coords, marker='o', linestyle='-', color='b', label='SLAM Path')
    ax.scatter(x_coords[0], y_coords[0], z_coords[0], color='g', s=100, label='Start')  # 起点
    ax.scatter(x_coords[-1], y_coords[-1], z_coords[-1], color='r', s=100, label='End')  # 终点

    # 添加标签和标题
    ax.set_title('3D SLAM Path Visualization')
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_zlabel('Z Position')
    ax.legend()
    ax.grid(True)

    # 显示图形
    plt.show()

# 文件路径
file_path = r'/home/gym/data/ros/slam_data3.txt'  # 替换为你的文件路径

# 绘制三维 SLAM Path
plot_3d_slam_path(file_path)