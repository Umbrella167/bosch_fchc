import rospy
from ui.Ui import UI  # 确保路径正确
from utils.ClientLogManager import client_logger
import subprocess
import signal
import os

roscore_process = None  # 全局变量，用于存储 roscore 的进程信息

def start_roscore():
    global roscore_process  # 声明为全局变量
    try:
        # 启动 roscore 并记录进程信息
        roscore_process = subprocess.Popen(['roscore'], preexec_fn=os.setsid)
    except Exception as e:
        print("Failed to start roscore:", e)

def stop_roscore():
    global roscore_process  # 声明为全局变量
    if roscore_process:
        try:
            # 终止 roscore 进程
            os.killpg(os.getpgid(roscore_process.pid), signal.SIGTERM)  # 发送终止信号
            roscore_process.wait()  # 等待进程终止
        except Exception as e:
            print("Failed to stop roscore:", e)
        finally:
            roscore_process = None  # 清空全局变量，表示 roscore 已被终止

def loop(UI):
    try:
        UI.update()
    except Exception as e:
        client_logger.log("ERROR", f"Loop Failed!", e)

def main():
    try:
        # 启动 roscore
        start_roscore()
        # 初始化 ROS 节点
        rospy.init_node('Client', anonymous=True)
        # 启动用户界面
        ui = UI()
        ui.show()
        ui.run_loop(lambda: loop(ui))
    finally:
        # 程序结束时关闭 roscore
        stop_roscore()

if __name__ == "__main__":
    main()