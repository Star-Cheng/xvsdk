import numpy as np

# 给定的三维坐标数据
# data = [
#     (5801.905762, -12659.284180, 32010.000000),
#     (5864.218262, -12659.382812, 32010.000000),
#     (5937.641113, -12683.210938, 32070.000000),
# ]
with open(r'/home/gym/code/edge/build/xvsdk/sgbm_demo/tof_pointcloud.txt', 'r') as f:
    data = f.readlines()
print(data[:10])
with open(r'/home/gym/code/edge/build/xvsdk/sgbm_demo/tof_pointcloud_new.txt', 'w') as w:
    for i in data[:]:
        print(i.split()[0].split("=")[-1], i.split()[1].split("=")[-1], i.split()[-1].split("=")[-1])
        x, y, z = i.split()[0].split("=")[-1], i.split()[1].split("=")[-1], i.split()[-1].split("=")[-1]
        w.write(str(x)+" "+str(y)+" "+str(z))
        w.write("\n")
