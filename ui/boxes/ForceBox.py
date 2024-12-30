import dearpygui.dearpygui as dpg
from ui.boxes.BaseBox import BaseBox
import rospy
import rospy.core
import rospy.msg
from geometry_msgs.msg import WrenchStamped,PoseStamped
from collections import deque
import time
import csv
from utils.ClientLogManager import client_logger
import os
class ForceBox(BaseBox):
    only = False

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.plot_x_tag = None
        self.plot_y_tag = None
        self.plot_fit = True
    def create(self):
        dpg.configure_item(self.tag, label="Plot Visualizer")
        with dpg.group(horizontal=True, parent=self.tag):
            button = dpg.add_button(label="Save Data")
        self.create_plot()
        self.monitor = FtMonitor(self.plot_x_tag, write_interval=1)
        dpg.configure_item(button,callback=self.monitor.save_to_csv,user_data=True)
        self.handler_registry()
        rospy.Subscriber('ft_sensor', WrenchStamped, self.monitor.ft_callback)
    def handler_registry(self):
        def f():
            if not dpg.does_item_exist(self.tag):
                return
            if not dpg.is_item_focused(self.tag):
                return
            self.plot_fit = not self.plot_fit
        with dpg.handler_registry():
            dpg.add_key_release_handler(
                key=dpg.mvKey_Spacebar,
                callback=f,
                user_data=self.plot_tag,
            )

    def create_plot(self):
        self.plot_tag = dpg.add_plot(
            width=-1,
            height=-1,
            label="Plot Tools",
            parent=self.tag,
            anti_aliased=True,
        )
        dpg.add_plot_legend(parent=self.plot_tag)
        dpg.add_plot_annotation(parent=self.plot_tag)
        self.plot_x_tag = dpg.add_plot_axis(dpg.mvXAxis, label="Time", parent=self.plot_tag)
        self.plot_y_tag = dpg.add_plot_axis(dpg.mvYAxis, label="Data", parent=self.plot_tag)
    def destroy(self):
        super().destroy()

    
    def update(self):
        if self.plot_fit:
            self.monitor.draw()
            dpg.fit_axis_data(self.plot_y_tag)
            dpg.fit_axis_data(self.plot_x_tag)

class FtMonitor():
    def __init__(self, plot_x_tag, write_interval=1):
        self.start_time = time.time()
        self.maxlen = 2000
        self.time_list = deque(maxlen=self.maxlen)
        self.Fx_list, self.Fy_list, self.Fz_list = deque(maxlen=self.maxlen), deque(maxlen=self.maxlen), deque(maxlen=self.maxlen)
        self.Mx_list, self.My_list, self.Mz_list = deque(maxlen=self.maxlen), deque(maxlen=self.maxlen), deque(maxlen=self.maxlen)
        self.fx_plot = dpg.add_line_series(x=[0], y=[0], parent=plot_x_tag, label="Fx")
        self.fy_plot = dpg.add_line_series(x=[0], y=[0], parent=plot_x_tag, label="Fy")
        self.fz_plot = dpg.add_line_series(x=[0], y=[0], parent=plot_x_tag, label="Fz")
        self.mx_plot = dpg.add_line_series(x=[0], y=[0], parent=plot_x_tag, label="Mx")
        self.my_plot = dpg.add_line_series(x=[0], y=[0], parent=plot_x_tag, label="My")
        self.mz_plot = dpg.add_line_series(x=[0], y=[0], parent=plot_x_tag, label="Mz")

        # CSV 文件相关
        self.count = 0
        self.csv_file_created = False
        self.csv_file_name = None
        self.write_interval = write_interval  # 写入间隔
        self.last_save_time = 0  # 记录上一次保存的时间点

    def ft_callback(self, msg):
        self.time_list.append(time.time() - self.start_time)
        self.Fx_list.append(msg.wrench.force.x)
        self.Fy_list.append(msg.wrench.force.y)
        self.Fz_list.append(msg.wrench.force.z)
        self.Mx_list.append(msg.wrench.torque.x)
        self.My_list.append(msg.wrench.torque.y)
        self.Mz_list.append(msg.wrench.torque.z)
        self.count += 1
        if self.count >= self.maxlen:
            self.save_to_csv()
            self.count = 0

    def read_csv(self, file_path, index_start=0, index_end=-1):
        try:
            # 打开 CSV 文件并读取数据
            with open(file_path, mode='r') as file:
                reader = csv.reader(file)
                data = list(reader)  # 转为列表以方便处理
                
                # 检查文件是否为空
                if not data:
                    raise ValueError("The CSV file is empty.")
                
                # 检查表头是否正确
                header = data[0]
                if header != ["Time", "Fx", "Fy", "Fz", "Mx", "My", "Mz"]:
                    raise ValueError("CSV file header does not match expected format.")
                
                # 提取数据部分，跳过表头
                data_rows = data[1:]
                
                # 处理索引
                index_end = len(data_rows) if index_end == -1 else index_end
                if index_start < 0 or index_end > len(data_rows) or index_start >= index_end:
                    raise ValueError("Invalid index range for reading CSV data.")
                
                # 创建新的列表
                time_list = []
                Fx_list = []
                Fy_list = []
                Fz_list = []
                Mx_list = []
                My_list = []
                Mz_list = []
                
                # 加载指定范围内的数据
                for row in data_rows[index_start:index_end]:
                    if len(row) != 7:
                        raise ValueError(f"CSV row has incorrect number of columns: {row}")
                    
                    # 将数据添加到新的列表中
                    time_list.append(float(row[0]))
                    Fx_list.append(float(row[1]))
                    Fy_list.append(float(row[2]))
                    Fz_list.append(float(row[3]))
                    Mx_list.append(float(row[4]))
                    My_list.append(float(row[5]))
                    Mz_list.append(float(row[6]))
            
            client_logger.log("success", f"Succeed to read data from CSV file: {file_path}")
            
            # 返回新的列表作为字典
            return {
                "time_list": time_list,
                "Fx_list": Fx_list,
                "Fy_list": Fy_list,
                "Fz_list": Fz_list,
                "Mx_list": Mx_list,
                "My_list": My_list,
                "Mz_list": Mz_list
            }
        except FileNotFoundError:
            client_logger.log("error", f"File not found: {file_path}")
            return None
        except IOError as e:
            client_logger.log("error", f"IOError while reading CSV file: {e}")
            return None
        except Exception as e:
            client_logger.log("error", f"Unexpected error: {e}")
            return None

    def save_to_csv(self, sender = None, app_data = None, user_data = None):
        """
        Saves the data to a CSV file. If finalize=True, move the CSV file to the logs/csv directory.
        """
        # 检查写入间隔
        if user_data is None:
            finalize=False 
        else:
            finalize = user_data
        
        if self.write_interval <= 0:
            raise ValueError("write_interval must be a positive integer")

        # 检查队列长度一致性
        assert len(self.time_list) == len(self.Fx_list) == len(self.Fy_list) == len(self.Fz_list) == len(self.Mx_list) == len(self.My_list) == len(self.Mz_list), "Data queues have inconsistent lengths"

        # 查找上一次保存的时间点索引
        start_index = 0
        for i, time_point in enumerate(self.time_list):
            if time_point > self.last_save_time:
                start_index = i
                break

        # 保存数据的结束索引
        end_index = len(self.time_list)  # 当前队列的长度

        # 更新 last_save_time
        self.last_save_time = self.time_list[-1] if len(self.time_list) > 0 else self.last_save_time

        # 如果当前文件不存在或者文件大小已经达到 1GB，创建新的文件
        if not self.csv_file_created or (os.path.exists(self.csv_file_name) and os.path.getsize(self.csv_file_name) >= 1 * 1024 * 1024 * 1024):  # 1GB = 1024 * 1024 * 1024 bytes
            # 检查 temp 文件夹中的文件数量
            temp_directory = "logs/csv/temp"
            temp_files = [os.path.join(temp_directory, f) for f in os.listdir(temp_directory) if os.path.isfile(os.path.join(temp_directory, f))]
            temp_files.sort(key=os.path.getctime)  # 按文件创建时间排序
            
            # 如果文件数量超过 3 个，删除最老的文件
            if len(temp_files) > 3:
                oldest_file = temp_files[0]
                try:
                    os.remove(oldest_file)
                    client_logger.log("success", f"Oldest temp file deleted: {oldest_file}")
                except Exception as e:
                    client_logger.log("error", f"Error while deleting old temp file: {e}")

            # 创建新的文件
            timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(time.time()))
            self.csv_file_name = os.path.join(temp_directory, f"ft_data_{timestamp}.csv")
            self.csv_file_created = True

        try:
            # 写入 CSV 文件
            with open(self.csv_file_name, mode='a', newline='') as file:
                writer = csv.writer(file)
                # 如果是第一次写入，添加表头
                if file.tell() == 0:  # 文件为空时写入表头
                    writer.writerow(["Time", "Fx", "Fy", "Fz", "Mx", "My", "Mz"])
                # 按照 start_index 和 end_index 写入数据
                for i in range(start_index, end_index):
                    # 按照写入间隔跳跃写入
                    if i % self.write_interval == 0:
                        writer.writerow([
                            self.time_list[i],
                            self.Fx_list[i],
                            self.Fy_list[i],
                            self.Fz_list[i],
                            self.Mx_list[i],
                            self.My_list[i],
                            self.Mz_list[i]
                        ])
            client_logger.log("success", f"Succeed to write data to CSV file: {self.csv_file_name}")

            # 如果是最终保存操作，将文件移动到 logs/csv 目录
            if finalize:
                # 创建目标目录（如果不存在）
                final_directory = "logs/csv"
                os.makedirs(final_directory, exist_ok=True)

                # 获取 temp 文件夹中的所有文件
                temp_directory = "logs/csv/temp"
                temp_files = [os.path.join(temp_directory, f) for f in os.listdir(temp_directory) if os.path.isfile(os.path.join(temp_directory, f))]
                
                # 遍历并移动所有文件到目标目录
                for temp_file in temp_files:
                    try:
                        # 生成目标文件路径
                        final_file_name = os.path.join(final_directory, os.path.basename(temp_file))
                        
                        # 移动文件
                        os.rename(temp_file, final_file_name)
                        
                        # 记录日志
                        client_logger.log("success", f"File moved to final directory: {final_file_name}")
                    except Exception as e:
                        client_logger.log("error", f"Error while moving file {temp_file}: {e}")
                
                # 清空 temp 文件夹：检查是否还有残留文件
                remaining_files = [os.path.join(temp_directory, f) for f in os.listdir(temp_directory) if os.path.isfile(os.path.join(temp_directory, f))]
                if not remaining_files:
                    client_logger.log("success", "Temp directory has been successfully cleared.")
                else:
                    client_logger.log("warning", f"Temp directory is not fully cleared. Remaining files: {remaining_files}")

        except IOError as e:
            client_logger.log("error", f"IOError while writing to CSV file: {e}")
        except Exception as e:
            client_logger.log("error", f"Unexpected error: {e}")
    def draw(self):
        for plot, data in zip(
            [self.fx_plot, self.fy_plot, self.fz_plot, self.mx_plot, self.my_plot, self.mz_plot], 
            [self.Fx_list, self.Fy_list, self.Fz_list, self.Mx_list, self.My_list, self.Mz_list]
        ):
            dpg.configure_item(plot, x=list(self.time_list), y=list(data))