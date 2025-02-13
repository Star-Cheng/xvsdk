import numpy as np
import re

# 从 txt 文件中读取数据
def read_data_from_txt(filename):
    points = []
    with open(filename, 'r') as f:
        for line in f:
            # 使用正则表达式提取 x, y, z 坐标值
            match = re.match(r'x=([-\d.]+) ,y=([-\d.]+) ,z=([-\d.]+)', line)
            if match:
                x = float(match.group(1))
                y = float(match.group(2))
                z = float(match.group(3))
                points.append((x, y, z))
    return np.array(points)

# 写入 PCD 文件
def write_pcd(filename, points):
    with open(filename, 'w') as f:
        # PCD 文件头
        f.write('# .PCD v7 - Point Cloud Data file format\n')
        f.write('VERSION .7\n')
        f.write('FIELDS x y z\n')
        f.write('SIZE 4 4 4\n')
        f.write('TYPE F F F\n')
        f.write('COUNT 1 1 1\n')
        f.write(f'POINTS {len(points)}\n')
        f.write('DATA ascii\n')
        
        # 写入点数据
        for point in points:
            f.write(f'{point[0]} {point[1]} {point[2]}\n')

# 从 txt 文件读取数据
points = read_data_from_txt(r'/home/gym/code/edge/build/xvsdk/sgbm_demo/tof_pointcloud.txt')

# 将数据保存为 pcd 文件
write_pcd('output.pcd', points)

print("PCD 文件已成功创建!")
