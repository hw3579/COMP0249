import json
import matplotlib.pyplot as plt

# 读取 JSON 文件
with open('./Labs/Lab_02_-_KF-Based_SLAM/Code/+l2/config/activity2-4/slam_landmarks.json', 'r') as file:
    data = json.load(file)

# 提取地标点
landmarks = data['landmarks']['slam']['landmarks']

# 分离 x 和 y 坐标
x_coords = [point[0] for point in landmarks]
y_coords = [point[1] for point in landmarks]

# 创建散点图
plt.scatter(x_coords, y_coords, marker='o', color='b')

# 添加标题和标签
plt.title('SLAM Landmarks')
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')

# 显示图形
plt.grid(True)
plt.show()