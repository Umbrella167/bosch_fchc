import dearpygui.dearpygui as dpg
from ui.boxes.BaseBox import BaseBox
from utils.ClientLogManager import client_logger
from config.SystemConfig import VCHISEL_WS_DIR
import xml.etree.ElementTree as ET

class ParamBox(BaseBox):
    only = False

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.launch_dir = VCHISEL_WS_DIR + "src/norm_calc/launch/norm_calc.launch"
        self.params_dict = self.read_xml(self.launch_dir)
        self.table_title = ["Param", "Value","Save","Info"]
        self.items_tag = {}
    def create_theme(self):
        with dpg.theme() as theme_id:
            with dpg.theme_component(dpg.mvAll):
                dpg.add_theme_color(
                    dpg.mvThemeCol_FrameBg,
                    (0, 0, 0, 0),
                    category=dpg.mvThemeCat_Core,
                )
                dpg.add_theme_color(
                    dpg.mvThemeCol_FrameBgHovered,
                    (0, 0, 0, 0),
                    category=dpg.mvThemeCat_Core,
                )
                dpg.add_theme_color(
                    dpg.mvThemeCol_FrameBgActive,
                    (0, 0, 0, 0),
                    category=dpg.mvThemeCat_Core,
                )
        return theme_id
    
    def create(self):
        dpg.configure_item(self.tag, label="Param Box")
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
        self.theme_id = self.create_theme()

        for t in self.table_title:
            dpg.add_table_column(label=t, width_fixed=True, parent=self.table_tag)
        for key in self.params_dict:
            self.items_tag[key] = {}
            row_tag = dpg.add_table_row(parent=self.table_tag)
            dpg.add_text(default_value=key, parent=row_tag)
            self.items_tag[key]["input_flot"]= dpg.add_drag_float(width=100,default_value=float(self.params_dict[key]), parent=row_tag,max_value=1e9,min_value=-1e9,speed=0.01)
            self.items_tag[key]["save_param"] = dpg.add_button(width=100,label="Save",parent=row_tag,callback=self.save_param,user_data=key)
            self.items_tag[key]["input_text_tag"] = dpg.add_input_text(width=999,default_value="", parent=row_tag)
            dpg.bind_item_theme(item=self.items_tag[key]["input_flot"], theme=self.theme_id)
            dpg.bind_item_theme(item=self.items_tag[key]["input_text_tag"], theme=self.theme_id)

    def save_param(self, sender, app_data, user_data):
        key = user_data
        if "last_value" not in self.items_tag[key]:
            for k in self.items_tag:
                self.items_tag[k]["last_value"] = dpg.get_value(self.items_tag[k]["input_flot"])


        dpg.set_value(self.items_tag[key]["input_text_tag"], f"Saved!, last value: {self.items_tag[key]['last_value']}")
        value = dpg.get_value(self.items_tag[key]["input_flot"])
        self.items_tag[key]["last_value"] = value
        self.params_dict[key] = value
        
        self.write_xml(self.launch_dir, self.params_dict)
    def write_xml(self, path, params_dict):
        """
        将更新后的参数写回到 XML 文件
        """
        tree = ET.parse(path)
        root = tree.getroot()

        # 遍历 XML 文件并更新参数值
        for param in root.findall(".//param"):
            name = param.get('name')
            if name in params_dict:
                param.set('value', str(params_dict[name]))

        # 将修改后的树写回到文件
        tree.write(path, encoding="utf-8", xml_declaration=True)
        client_logger.log("success",f"Updated parameters saved to {path}")

    def read_xml(self,path):
        tree = ET.parse(self.launch_dir)
        root = tree.getroot()
        params_dict = {}

        # 查找所有的 <param> 节点
        for param in root.findall(".//param"):
            # 获取 name 和 value 属性
            name = param.get('name')
            value = param.get('value')
            if name and value:  # 确保 name 和 value 存在
                params_dict[name] = value
        return params_dict
    
    def set_input_color(change_item, color):
        with dpg.theme() as theme_id:
            with dpg.theme_component(dpg.mvAll):
                dpg.add_theme_color(dpg.mvThemeCol_FrameBg, (0, 0, 0, 0), category=dpg.mvThemeCat_Core)
                dpg.add_theme_color(
                    dpg.mvThemeCol_FrameBgHovered,
                    (0, 0, 0, 0),
                    category=dpg.mvThemeCat_Core,
                )
                dpg.add_theme_color(
                    dpg.mvThemeCol_FrameBgActive,
                    (0, 0, 0, 0),
                    category=dpg.mvThemeCat_Core,
                )
                dpg.add_theme_color(dpg.mvThemeCol_Text, color)
        dpg.bind_item_theme(item=change_item, theme=theme_id)


    def destroy(self):
        super().destroy()
    
    def update(self):
        pass