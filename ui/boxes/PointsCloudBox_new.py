import dearpygui.dearpygui as dpg
from ui.boxes.BaseBox import BaseBox
from utils.ClientLogManager import client_logger
from config.SystemConfig import VCHISEL_WS_DIR
import pyvista as pv
import numpy as np
from ui.components.Canvas2D import Canvas2D
import threading
import cv2
class PointsCloudBox(BaseBox):
    only = False
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.size = (1024,768)
        self.point_cloud_img = None
        t = threading.Thread(target=self.draw_thread)
        t.start()
        self.last_mouse_pos = (0, 0)
        self.now_mouse_pos = (0, 0)
        self.zoom = 1
        self.dx = 0
        self.dy = 0
    def create(self):
        dpg.configure_item(self.tag, label="PointsCloud Box")
        self.canvas = Canvas2D(self.tag,auto_mouse_transfrom=False)
        self.texture_id = self.canvas.texture_register(self.size)
        with self.canvas.draw() as draw_tag:
            dpg.draw_image(self.texture_id, [0, 0], self.size)
        self.handler_register()
        
    def draw_thread(self):
        n_points = 1000
        points = np.random.rand(n_points, 3) # 随机生成100个3维点

        self.point_cloud = pv.PolyData(points)
        self.plotter = pv.Plotter(off_screen=True)
        self.plotter.add_points(self.point_cloud, color="blue", point_size=5)
        
        while True:
            points += (np.random.rand(n_points, 3) - 0.5) * 0.01
            self.point_cloud_img = self.plotter.screenshot()
            self.plotter.update()
    def handler_register(self):
        with dpg.handler_registry():
            dpg.add_mouse_move_handler(callback=self.mouse_drag_callback)
            dpg.add_mouse_move_handler(callback=self.mouse_drag_callback)
            dpg.add_mouse_wheel_handler(callback=self.mouse_wheel_callback)
            
    def mouse_drag_callback(self, sender, app_data,user_data):
        self.last_mouse_pos = self.now_mouse_pos
        self.now_mouse_pos = dpg.get_drawing_mouse_pos()
        dx = self.now_mouse_pos[0] - self.last_mouse_pos[0]
        dy = self.now_mouse_pos[1] - self.last_mouse_pos[1]
        
        
        if dpg.is_mouse_button_down(dpg.mvMouseButton_Left):
            self.dx += dx
            self.dy += dy
            print(self.dx,dx,self.dy,dy)
            self.update_camera(dx=-self.dx, dy=self.dy, rotate=True)

        elif dpg.is_mouse_button_down(dpg.mvMouseButton_Right):
            self.dx += dx
            self.dy += dy
            self.update_camera(dx=-self.dx, dy=self.dy, pan=True)

    def mouse_wheel_callback(self, sender, app_data):
        """
        鼠标滚轮事件回调
        """
        zoom_factor = app_data * 0.1
        self.zoom += zoom_factor
        self.update_camera(zoom=zoom_factor)

    def update_camera(self, dx=0, dy=0, dz=0, zoom=0, pan=False, rotate=False):
        """
        更新相机视角（平移、旋转或缩放）。
        dx, dy, dz: 平移的偏移量。
        zoom: 缩放因子。
        pan: 是否进行平移操作。
        rotate: 是否进行旋转操作。
        """
        if zoom != 0:
            # 缩放
            self.plotter.camera.zoom(1 + zoom)

        if pan:
            pass
        if rotate:
            self.plotter.camera.azimuth = dx * 0.2
            self.plotter.camera.elevation = dy * 0.2
            
    def destroy(self):
        # 确保在销毁前终止 ROS 进程
        self.close()
        super().destroy()
    def update(self):
        if self.point_cloud_img is not None:
            self.canvas.texture_update(self.texture_id, self.point_cloud_img)