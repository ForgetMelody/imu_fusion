import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

# 读取txt文件中的点数据
def read_txt_file(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
        points = []
        for line in lines:
            data = line.strip().split()
            points.append([float(data[0]), float(data[1]), float(data[2])])
    return np.array(points)

# 读取多个txt文件
def read_txt_files(folder_path):
    file_paths = [os.path.join(folder_path, file) for file in os.listdir(folder_path) if file.endswith('.txt')]
    trajectories = []
    file_names = []
    for file_path in file_paths:
        points = read_txt_file(file_path)
        trajectories.append(points)
        file_names.append(os.path.basename(file_path).split('.')[0])
    return trajectories, file_names

# 绘制轨迹
def plot_trajectories(trajectories, file_names):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    colors = plt.cm.viridis(np.linspace(0, 1, len(trajectories)))  # 生成不同颜色
    
    for i, (trajectory, file_name) in enumerate(zip(trajectories, file_names)):
        ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], color=colors[i], label=file_name)
 
        # 添加标签
        x_label, y_label, z_label = trajectory[-1, 0], trajectory[-1, 1], trajectory[-1, 2]
        ax.text(x_label, y_label, z_label, file_name, color='black')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend(loc='upper left')
    plt.show()

if __name__ == '__main__':
    folder_path = "/home/sosilent/catkin_ws/data/paths/"  # 指定包含txt文件的文件夹路径
    trajectories, file_names = read_txt_files(folder_path)
    plot_trajectories(trajectories, file_names)
