import numpy as np
import pyvista as pv
import time
import cv2

# 创建 PyVista 的 Plotter
plotter = pv.Plotter(off_screen=True)

# 初始化点云数据
n_points = 1000
points = np.random.rand(n_points, 3)  # 随机生成初始点云

# 创建点云的 PolyData
point_cloud = pv.PolyData(points)

# 在绘图器中添加点云数据
plotter.add_points(point_cloud, color="blue", point_size=5)

# 设置交互模式
plotter.show(auto_close=False)

# 实时动态添加点
while True:
    # 动态添加新点
    new_points = np.random.rand(10, 3)  # 每次添加 10 个随机点
    points = np.vstack((points, new_points))  # 将新点添加到现有点云中
    point_cloud.points = points  # 更新点云数据

    # 通知绘图器更新
    plotter.update()

    # 截图并用 OpenCV 显示
    res = plotter.screenshot()
    cv2.imshow("Point Cloud", res)
    if cv2.waitKey(1) & 0xFF == ord('q'):  # 按 'q' 键退出
        break

# 释放资源
cv2.destroyAllWindows()