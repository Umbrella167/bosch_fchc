import subprocess
import dearpygui.dearpygui as dpg
from ui.boxes.BaseBox import BaseBox
from utils.ClientLogManager import client_logger
from config.SystemConfig import VCHISEL_WS_DIR
import rospy
from rosgraph_msgs.msg import Log
class OutPutBox(BaseBox):
    only = False
    process = None  # 用于存储启动的进程对象

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.info_sub = rospy.Subscriber(
                        "/rosout", 
                        Log, 
                        callback=self.f_msg
                    )
        
        self.info_all = ""
    def f_msg(self, msg):
        print(msg)
        # 提取关键字段
        log_level_map = {
            1: "DEBUG",
            2: "INFO",
            4: "WARN",
            8: "ERROR",
            16: "FATAL"
        }
        
        # 格式化日志内容
        log_level = log_level_map.get(msg.level, "UNKNOWN")  # 获取日志级别
        timestamp = f"{msg.header.stamp.secs}.{msg.header.stamp.nsecs:09d}"  # 格式化时间戳
        name = msg.name  # 节点名称
        message = msg.msg  # 日志消息
        file_name = msg.file  # 触发日志的文件
        function = msg.function  # 触发日志的函数
        line = msg.line  # 行号
        topics = ", ".join(msg.topics)  # 订阅的主题

        # 构建格式化日志字符串
        formatted_log = (
            f"[{timestamp}] [{log_level}] [{name}] {message}\n"
            f"    File: {file_name}, Function: {function}, Line: {line}\n"
            f"    Topics: {topics}\n"
        )
        self.info_all += formatted_log + "\n"
        dpg.set_value(self.input_text, self.info_all)
    def create(self):
        dpg.configure_item(self.tag, label="OutPutBox")
        self.input_text = dpg.add_input_text( parent=self.tag,multiline=True, height=-1, width=-1)


    def destroy(self):
        # 在销毁时提示用户关闭终端
        super().destroy()