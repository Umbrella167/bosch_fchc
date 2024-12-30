import subprocess
import os
import signal
import dearpygui.dearpygui as dpg
from ui.boxes.BaseBox import BaseBox
from utils.ClientLogManager import client_logger
from config.SystemConfig import VCHISEL_WS_DIR
import psutil
class ModbusBox(BaseBox):
    only = False
    process = None  # 用于存储启动的进程对象

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def create(self):
        dpg.configure_item(self.tag, label="Modbus Box")
        with dpg.group(horizontal=True, parent=self.tag):
            dpg.add_button(label="Start", width=100, height=100, callback=self.start)
            dpg.add_button(label="Close", width=100, height=100, callback=self.close)

    def start(self):
        # 构建命令
        cmd = f"cd {VCHISEL_WS_DIR} && source devel/setup.sh && roslaunch modbus Core.launch"
        try:
            # 打开一个新的终端并运行命令，保存进程对象
            self.process = subprocess.Popen(["gnome-terminal", "--", "bash", "-c", cmd])
        except Exception as e:
            client_logger.log("error",f"Failed to start terminal: {e}")
    def kill_ros_processes(self):
        try:
            # 遍历所有进程
            for process in psutil.process_iter(attrs=['pid', 'name', 'cmdline']):
                try:
                    # 检查进程的命令行是否包含 "ros"
                    cmdline = process.info['cmdline']
                    if cmdline and any("ros" in arg for arg in cmdline):
                        client_logger.log("info", f"Killing ROS process: {cmdline}")
                        process.terminate()  # 尝试优雅地终止进程
                        process.wait(timeout=5)  # 等待进程终止
                        if process.is_running():  # 如果进程仍在运行，则强制杀死
                            process.kill()
                except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                    # 如果进程不存在或没有权限，跳过
                    continue

            client_logger.log("info", "All ROS processes terminated successfully.")
        except Exception as e:
            client_logger.log("error", f"Failed to terminate ROS processes: {e}")


    def close(self):
        self.kill_ros_processes()
        # 强制关闭终端及其子进程
        try:
            # 获取终端的子进程树并逐一终止
            parent_pid = self.process.pid  # 获取终端进程 PID
            parent_process = psutil.Process(parent_pid)  # 获取父进程对象
            children = parent_process.children(recursive=True)  # 获取所有子进程


            # 如果子进程未能关闭，强制杀死它们
            for child in children:
                if child.is_running():
                    child.kill()

            # 最后关闭父进程（终端本身）
            parent_process.terminate()
            parent_process.wait(timeout=5)
            if parent_process.is_running():
                parent_process.kill()

            self.process = None  # 清空进程对象
            client_logger.log("info", "Terminal and its subprocesses closed successfully.")
        except Exception as e:
            client_logger.log("error", f"Failed to close terminal: {e}")

    def destroy(self):
        # 在销毁时提示用户关闭终端
        self.close()
        super().destroy()