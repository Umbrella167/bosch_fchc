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
        
        self.COLOR_MAP = {
            "ERROR": (220, 53, 69, 100),   # 柔和的红色
            "DEBUG": (23, 162, 184, 100), # 柔和的蓝绿色
            "INFO": (40, 167, 69, 100),   # 柔和的绿色
            "WARN": (255, 193, 7, 100),   # 柔和的黄色
            "FATAL": (111, 66, 193, 100)  # 柔和的紫色
        }
        self.msg_count = 0
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
        self.add_msg(line,timestamp,log_level,name,message,file_name,function,topics)
    def create(self):
        dpg.configure_item(self.tag, label="OutPutBox")
        self.create_table()
        
    def create_table(self):
        self.table_tag = dpg.add_table(
            header_row=True,
            policy=dpg.mvTable_SizingFixedFit,
            row_background=True,
            reorderable=True,
            resizable=True,
            no_host_extendX=False,
            hideable=True,
            borders_innerV=True,
            delay_search=True,
            borders_outerV=True,
            borders_innerH=True,
            borders_outerH=True,
            parent=self.tag,
            height= -1,
            width=-1,
        )
        info = ["Index","Line", "Timestamp", "Level", "Name", "Message", "File", "Function", "Topics"]
        for t in info:
            dpg.add_table_column(label=t, width_fixed=True, parent=self.table_tag)
    def add_msg(self, line,timestamp,log_level,name,message,file_name,function,topics):
        infos = [self.msg_count,line,timestamp,log_level,name,message,file_name,function,topics]
        color = self.COLOR_MAP[log_level]
        row_tag = dpg.add_table_row(parent=self.table_tag)
        for count,info in enumerate(infos):
            dpg.add_selectable(label=info, span_columns=True, parent=row_tag,tracked = True,track_offset = 1.0)
        dpg.highlight_table_row(self.table_tag,self.msg_count , color)
        dpg.set_y_scroll(self.table_tag, self.msg_count * 1000)
        self.msg_count += 1
        
    def destroy(self):
        super().destroy()
        
    def update(self):
        pass