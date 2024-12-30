import dearpygui.dearpygui as dpg
from ui.boxes.BaseBox import BaseBox
from utils.ClientLogManager import client_logger
from config.SystemConfig import VCHISEL_WS_DIR
import os
import subprocess
import signal
import psutil

class PointsCloudBox(BaseBox):
    only = False

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.ros_process = None  # 用于存储启动的进程

    def create(self):
        dpg.configure_item(self.tag, label="PointsCloud Box")
        with dpg.group(horizontal=True, parent=self.tag):
            dpg.add_button(label="Start", width=100, height=100, callback=self.start)
            dpg.add_button(label="Close", width=100, height=100, callback=self.close)

    def start(self):
        try:
            if self.ros_process is not None:
                client_logger.log("ROS process is already running.")
                return

            # 启动 norm_viewer 并记录进程信息
            cmd = f"bash -c 'cd {VCHISEL_WS_DIR} && source devel/setup.sh && roslaunch norm_calc norm_viewer.launch'"

            self.ros_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                shell=True,
                preexec_fn=os.setsid  # 为子进程创建新的会话组
            )
            
            client_logger.log("success", "Started ROS process successfully.")
        except subprocess.TimeoutExpired:
            client_logger.log("info", "Process started, but no immediate output.")
        except Exception as e:
            client_logger.log("error",f"Failed to start ROS process: {e}")
            self.ros_process = None

    def close(self):
        try:
            if self.ros_process is not None:

                # 获取所有子进程并终止
                parent = psutil.Process(self.ros_process.pid)
                for child in parent.children(recursive=True):
                    child.terminate()  # 发送 SIGTERM
                gone, alive = psutil.wait_procs(parent.children(recursive=True), timeout=5)
                for p in alive:
                    p.kill()  # 强制终止未退出的子进程
                
                # 终止主进程组
                os.killpg(os.getpgid(self.ros_process.pid), signal.SIGKILL)
                self.ros_process = None
                client_logger.log("success", "ROS process and its children terminated successfully.")
            else:
                client_logger.log("info","No ROS process is running.")
        except Exception as e:
            client_logger.log("error", f"Failed to terminate ROS process: {e}")

    def destroy(self):
        # 确保在销毁前终止 ROS 进程
        self.close()
        super().destroy()

    def update(self):
        pass