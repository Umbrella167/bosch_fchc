import numpy as np
import pyvista as pv
import time
import cv2
# 创建 PyVista 的 Plotter
plotter = pv.Plotter()

# 初始化点云数据
n_points = 1000
points = np.random.rand(n_points, 3)  # 随机生成初始点云

# 创建点云的 PolyData
point_cloud = pv.PolyData(points)

# 在绘图器中添加点云数据
plotter.add_points(point_cloud, color="blue", point_size=5)

# 设置交互模式
plotter.show(auto_close=False)

# 实时更新点云
while True:
    # 更新点云的坐标（例如，随机移动点云）
    points += (np.random.rand(n_points, 3) - 0.5) * 0.01
    point_cloud.points = points  # 更新点云数据
    plotter.update()  # 通知绘图器更新
    time.sleep(0.05)  # 控制更新速度
    res = plotter.screenshot()
    cv2.imshow("res", res)
    cv2.waitKey(1)