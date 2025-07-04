import sys
import struct
import threading
import time
from datetime import datetime
from collections import deque
import numpy as np
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                            QWidget, QComboBox, QLabel, QPushButton, QSpinBox, 
                            QGroupBox, QGridLayout, QCheckBox)
from PyQt5.QtCore import QTimer, pyqtSignal, QObject, Qt
from PyQt5.QtGui import QFont
import pyqtgraph as pg

class DataReceiver(QObject):
    """串口数据接收器"""
    data_received = pyqtSignal(list)  # 发送接收到的8路数据
    status_changed = pyqtSignal(str)  # 发送状态信息
    
    def __init__(self, skip_mask=0b00000000):
        super().__init__()
        self.serial_port = None
        self.is_running = False
        self.thread = None
        self.skip_mask = skip_mask  # 8位二进制掩码，第n位为1表示跳过检查第n+1通道是否非零
        
    def set_skip_mask(self, mask):
        """设置跳过检查掩码
        Args:
            mask: 8位二进制掩码，第n位为1表示跳过检查第n+1通道是否非零
                  例如：0b00010000 表示跳过检查第5通道的数据
        """
        self.skip_mask = mask
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        print(f"[{timestamp}] 设置跳过检查掩码: {bin(mask)} (跳过通道: {self._get_skipped_channels()})")
    
    def _get_skipped_channels(self):
        """获取被跳过检查的通道列表"""
        skipped_channels = []
        for i in range(8):
            if (self.skip_mask >> i) & 1:
                skipped_channels.append(i + 1)
        return skipped_channels
    
    def _should_check_channel(self, channel_index):
        """判断是否应该检查该通道是否非零
        Args:
            channel_index: 通道索引（0-7）
        Returns:
            bool: True表示应该检查，False表示跳过检查
        """
        return not ((self.skip_mask >> channel_index) & 1)
        
    def start_receiving(self, port, baudrate, timeout=1):
        """开始接收数据"""
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=timeout)
            self.is_running = True
            self.thread = threading.Thread(target=self._receive_loop)
            self.thread.daemon = True
            self.thread.start()
            self.status_changed.emit(f"已连接到 {port}，波特率: {baudrate}")
            return True
        except Exception as e:
            self.status_changed.emit(f"连接失败: {str(e)}")
            return False
    
    def stop_receiving(self):
        """停止接收数据"""
        self.is_running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.status_changed.emit("已断开连接")
    
    def _receive_loop(self):
        """数据接收循环"""
        buffer = b""  # 使用字节缓冲区
        while self.is_running:
            try:
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    buffer += data
                    
                    # 查找完整的数据包（帧头0x55 + 8个int32 + 帧尾0xAA = 1 + 32 + 1 = 34字节）
                    while len(buffer) >= 34:
                        # 查找帧头0x55
                        start_pos = buffer.find(b'\x55')
                        if start_pos != -1:
                            # 检查是否有完整的数据包
                            if start_pos + 34 <= len(buffer):
                                packet_data = buffer[start_pos:start_pos + 34]
                                # 验证帧尾是否为0xAA
                                if packet_data[33] == 0xAA:
                                    # 提取8个int32数据（跳过帧头和帧尾）
                                    data_payload = packet_data[1:33]
                                    self._parse_data_packet(data_payload)
                                    buffer = buffer[start_pos + 34:]  # 移除已处理的数据包
                                else:
                                    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                                    print(f"[{timestamp}] 帧尾错误: 期望0xAA，得到0x{packet_data[33]:02X}")
                                    # 帧尾不匹配，继续寻找下一个帧头
                                    buffer = buffer[start_pos + 1:]
                            else:
                                # 数据不够，等待更多数据
                                break
                        else:
                            # 没有找到帧头，清除部分数据避免内存溢出
                            if len(buffer) > 100:
                                buffer = buffer[-33:]  # 保留可能的不完整帧
                            break
                else:
                    time.sleep(0.001)  # 避免CPU占用过高
            except Exception as e:
                self.status_changed.emit(f"数据接收错误: {str(e)}")
                break
    
    def _parse_data_packet(self, packet_data):
        """解析数据包"""
        try:
            # 解析32字节的数据为8个int32（小端字节序）
            if len(packet_data) == 32:
                parsed_data = list(struct.unpack('<8i', packet_data))
                
                # 检查需要检查的通道是否有非零数据
                should_print = False
                for i in range(8):
                    # 如果通道应该被检查且数据非零，则打印
                    if self._should_check_channel(i) and parsed_data[i] != 0:
                        should_print = True
                        break
                
                if should_print:
                    # 获取当前时间戳，精确到毫秒
                    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                    
                    # 构建打印信息，格式为[000,000,000,000,000,000,000,000]
                    print_data = []
                    for i in range(8):
                        if self._should_check_channel(i):
                            # 限制数值显示范围，超过1000显示999，小于-1000显示-999
                            value = parsed_data[i]
                            if value > 1000:
                                display_value = "999"
                            elif value < -1000:
                                display_value = "-999"
                            else:
                                display_value = f"{value:03d}" if value >= 0 else f"{value:04d}"
                            print_data.append(display_value)
                        else:
                            print_data.append("***")  # 跳过检查的通道用***表示
                    
                    print(f"[{timestamp}] 接收到数据: [{','.join(print_data)}]")
                    
                self.data_received.emit(parsed_data)
        except Exception as e:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            print(f"[{timestamp}] 数据解析错误: {e}")
            pass


class OscilloscopeWidget(QWidget):
    """示波器显示控件"""
    
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.init_data()
        
    def init_ui(self):
        """初始化用户界面"""
        layout = QVBoxLayout()
        
        # 创建绘图控件并设置白色背景
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot_widget.setBackground('w')  # 设置白色背景
        layout.addWidget(self.plot_widget)
        
        # 创建单个图表显示前4路数据
        self.plot = self.plot_widget.addPlot(title="ESP32 前四路数据显示 (通道1-4)")
        self.plot.setLabel('left', '幅值', color='black', size='12pt')
        self.plot.setLabel('bottom', '时间 (采样点)', color='black', size='12pt')
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        # 设置初始的Y轴范围
        self.plot.setYRange(-5000, 5000)  # 适中的初始范围
        self.plot.enableAutoRange(axis='x')  # 启用X轴自动范围调整
        # 不启用Y轴自动范围，让用户手动控制
        
        # 设置图表背景和边框颜色
        self.plot.getViewBox().setBackgroundColor('w')
        
        # 定义4种不同的颜色（只显示前4个通道）
        colors = [
            '#FF0000',  # 红色 - 通道1
            '#00AA00',  # 绿色 - 通道2
            '#0000FF',  # 蓝色 - 通道3
            '#FF8800',  # 橙色 - 通道4
        ]
        
        # 创建4条曲线，只显示前4个通道
        self.curves = []
        for i in range(4):  # 只创建4条曲线
            # 创建曲线但不初始化数据
            curve = self.plot.plot(
                [], [],  # 空数据
                pen=pg.mkPen(color=colors[i], width=2),
                name=f"通道 {i+1}"
            )
            self.curves.append(curve)
            print(f"创建通道{i+1}，颜色: {colors[i]}")
        
        # 为了保持数据处理的兼容性，补充4个空的曲线占位符
        for i in range(4, 8):
            self.curves.append(None)  # 占位符，不显示
        
        # 添加图例（在创建所有曲线后）
        try:
            legend = self.plot.addLegend()
            legend.setOffset((10, 10))
        except:
            # 如果图例添加失败，忽略错误
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            print(f"[{timestamp}] 图例添加失败，继续运行")
        
        self.setLayout(layout)
    
    def init_data(self):
        """初始化数据存储"""
        self.max_points = 1000  # 最大显示点数
        self.data_buffers = [deque(maxlen=self.max_points) for _ in range(8)]
        self.time_axis = list(range(self.max_points))
    
    def update_data(self, new_data):
        """更新波形数据"""
        # 调试计数
        if not hasattr(self, 'update_count'):
            self.update_count = 0
        self.update_count += 1
        
        # 检查通道1,2,3,4是否不为0（索引0,1,2,3）
        channels_to_check = [0, 1, 2, 3]  # 通道1,2,3,4
        has_non_zero = any(new_data[i] != 0 for i in channels_to_check)
    
        
        # 更新所有8个通道的数据缓冲区
        for i, value in enumerate(new_data):
            self.data_buffers[i].append(value)
            
        # 只更新前4个通道的显示
        for i in range(4):
            if self.curves[i] is not None:
                # 获取数据
                y_data = list(self.data_buffers[i])
                x_data = list(range(len(y_data)))
                
                # 设置曲线数据
                self.curves[i].setData(x_data, y_data)
    
    def clear_data(self):
        """清除所有数据"""
        for buffer in self.data_buffers:
            buffer.clear()
        # 只清除前4个通道的显示
        for i in range(4):
            if self.curves[i] is not None:
                self.curves[i].setData([], [])
    
    def auto_scale_y_axis(self):
        """自动缩放Y轴到合适的范围"""
        all_values = []
        # 只考虑前4个通道的数据进行自动缩放
        for i in range(4):
            if self.data_buffers[i]:
                all_values.extend(list(self.data_buffers[i]))
        
        if all_values:
            min_val = min(all_values)
            max_val = max(all_values)
            # 添加10%的边距
            margin = (max_val - min_val) * 0.1
            self.plot.setYRange(min_val - margin, max_val + margin)
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            print(f"[{timestamp}] Y轴范围调整为: {min_val - margin} 到 {max_val + margin}")
        else:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            print(f"[{timestamp}] 没有数据可用于自动缩放")


class ControlPanel(QWidget):
    """控制面板"""
    
    connect_clicked = pyqtSignal(str, int)  # 端口，波特率
    disconnect_clicked = pyqtSignal()
    
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.refresh_ports()
        
    def init_ui(self):
        """初始化控制面板界面"""
        layout = QVBoxLayout()
        
        # 串口设置组
        serial_group = QGroupBox("串口设置")
        serial_layout = QGridLayout()
        
        # COM端口选择
        serial_layout.addWidget(QLabel("端口:"), 0, 0)
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(100)
        serial_layout.addWidget(self.port_combo, 0, 1)
        
        refresh_btn = QPushButton("刷新")
        refresh_btn.clicked.connect(self.refresh_ports)
        serial_layout.addWidget(refresh_btn, 0, 2)
        
        # 波特率选择
        serial_layout.addWidget(QLabel("波特率:"), 1, 0)
        self.baudrate_combo = QComboBox()
        self.baudrate_combo.addItems(['9600', '115200', '230400', '460800', '921600'])
        self.baudrate_combo.setCurrentText('115200')
        serial_layout.addWidget(self.baudrate_combo, 1, 1)
        
        # 连接按钮
        self.connect_btn = QPushButton("连接")
        self.connect_btn.clicked.connect(self.on_connect_clicked)
        serial_layout.addWidget(self.connect_btn, 2, 0, 1, 3)
        
        serial_group.setLayout(serial_layout)
        layout.addWidget(serial_group)
        
        # 显示设置组
        display_group = QGroupBox("显示设置")
        display_layout = QGridLayout()
        
        # 显示点数设置
        display_layout.addWidget(QLabel("显示点数:"), 0, 0)
        self.points_spinbox = QSpinBox()
        self.points_spinbox.setRange(100, 5000)
        self.points_spinbox.setValue(1000)
        self.points_spinbox.setSuffix(" 点")
        display_layout.addWidget(self.points_spinbox, 0, 1)
        
        # 清除数据按钮
        clear_btn = QPushButton("清除数据")
        clear_btn.clicked.connect(self.clear_data_clicked)
        display_layout.addWidget(clear_btn, 1, 0, 1, 2)
        
        # 自动缩放按钮
        auto_scale_btn = QPushButton("自动缩放")
        auto_scale_btn.clicked.connect(self.auto_scale_clicked)
        display_layout.addWidget(auto_scale_btn, 2, 0, 1, 2)
        
        display_group.setLayout(display_layout)
        layout.addWidget(display_group)
        
        # 状态显示
        self.status_label = QLabel("未连接")
        self.status_label.setStyleSheet("QLabel { color: red; font-weight: bold; }")
        layout.addWidget(self.status_label)
        
        layout.addStretch()
        self.setLayout(layout)
        
        self.is_connected = False
        
    def refresh_ports(self):
        """刷新可用串口列表"""
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(f"{port.device} - {port.description}")
    
    def on_connect_clicked(self):
        """连接/断开按钮点击处理"""
        if not self.is_connected:
            if self.port_combo.count() > 0:
                port_text = self.port_combo.currentText()
                port = port_text.split(' - ')[0]  # 提取端口名
                baudrate = int(self.baudrate_combo.currentText())
                self.connect_clicked.emit(port, baudrate)
        else:
            self.disconnect_clicked.emit()
    
    def set_connected_state(self, connected):
        """设置连接状态"""
        self.is_connected = connected
        self.connect_btn.setText("断开" if connected else "连接")
        self.port_combo.setEnabled(not connected)
        self.baudrate_combo.setEnabled(not connected)
        
        if connected:
            self.status_label.setText("已连接")
            self.status_label.setStyleSheet("QLabel { color: green; font-weight: bold; }")
        else:
            self.status_label.setText("未连接")
            self.status_label.setStyleSheet("QLabel { color: red; font-weight: bold; }")
    
    clear_data_clicked = pyqtSignal()
    auto_scale_clicked = pyqtSignal()


class OscilloscopeMainWindow(QMainWindow):
    """主窗口"""
    
    def __init__(self, skip_mask=0b00000000):
        super().__init__()
        self.skip_mask = skip_mask
        self.init_ui()
        self.init_receiver()
        self.init_timer()
        
    def init_ui(self):
        """初始化主界面"""
        self.setWindowTitle("ESP32 前四路示波器上位机 (通道1-4)")
        self.setGeometry(100, 100, 1200, 800)
        
        # 创建中央控件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 创建主布局
        main_layout = QHBoxLayout()
        
        # 左侧控制面板
        self.control_panel = ControlPanel()
        self.control_panel.setMaximumWidth(250)
        self.control_panel.connect_clicked.connect(self.on_connect)
        self.control_panel.disconnect_clicked.connect(self.on_disconnect)
        self.control_panel.clear_data_clicked.connect(self.on_clear_data)
        self.control_panel.auto_scale_clicked.connect(self.on_auto_scale)
        main_layout.addWidget(self.control_panel)
        
        # 右侧示波器显示
        self.oscilloscope = OscilloscopeWidget()
        main_layout.addWidget(self.oscilloscope)
        
        central_widget.setLayout(main_layout)
        
        # 设置样式
        self.setStyleSheet("""
            QMainWindow {
                background-color: white;
            }
            QWidget {
                background-color: white;
            }
            QGroupBox {
                font-weight: bold;
                border: 2px solid #cccccc;
                border-radius: 5px;
                margin: 3px;
                padding-top: 10px;
                background-color: white;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
                color: black;
            }
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                padding: 8px 16px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:pressed {
                background-color: #3d8b40;
            }
            QLabel {
                color: black;
            }
            QComboBox {
                background-color: white;
                border: 1px solid #cccccc;
                border-radius: 3px;
                padding: 5px;
                color: black;
            }
            QSpinBox {
                background-color: white;
                border: 1px solid #cccccc;
                border-radius: 3px;
                padding: 5px;
                color: black;
            }
        """)
    
    def init_receiver(self):
        """初始化数据接收器"""
        self.data_receiver = DataReceiver(self.skip_mask)
        self.data_receiver.data_received.connect(self.on_data_received)
        self.data_receiver.status_changed.connect(self.on_status_changed)
    
    def init_timer(self):
        """初始化定时器"""
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_display)
        self.pending_data = []
        self.data_lock = threading.Lock()
    
    def on_connect(self, port, baudrate):
        """连接串口"""
        if self.data_receiver.start_receiving(port, baudrate):
            self.control_panel.set_connected_state(True)
            self.update_timer.start(50)  # 20Hz更新频率
    
    def on_disconnect(self):
        """断开连接"""
        self.update_timer.stop()
        self.data_receiver.stop_receiving()
        self.control_panel.set_connected_state(False)
    
    def on_data_received(self, data):
        """接收到新数据"""
        with self.data_lock:
            self.pending_data.append(data)
    
    def update_display(self):
        """更新显示"""
        with self.data_lock:
            if self.pending_data:
                # 只显示最新的数据
                latest_data = self.pending_data[-1]
                self.oscilloscope.update_data(latest_data)
                self.pending_data.clear()
    
    def on_status_changed(self, status):
        """状态改变处理"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        print(f"[{timestamp}] 状态: {status}")
    
    def on_clear_data(self):
        """清除数据"""
        self.oscilloscope.clear_data()
    
    def on_auto_scale(self):
        """自动缩放Y轴"""
        self.oscilloscope.auto_scale_y_axis()
    
    def closeEvent(self, event):
        """窗口关闭事件"""
        self.on_disconnect()
        event.accept()


def main():
    import argparse
    
    # 创建命令行参数解析器
    parser = argparse.ArgumentParser(description='ESP32 前四路示波器上位机')
    parser.add_argument('--skip-mask', type=str, default='0b00000000',
                        help='跳过检查掩码（8位二进制），第n位为1表示跳过检查第n+1通道是否非零。例如：0b00010000表示跳过检查第5通道')
    
    args = parser.parse_args()
    
    # 解析跳过检查掩码
    try:
        if args.skip_mask.startswith('0b'):
            skip_mask = int(args.skip_mask, 2)
        else:
            skip_mask = int(args.skip_mask)
        
        # 确保掩码在有效范围内
        skip_mask = skip_mask & 0xFF  # 限制为8位
        
        skipped_channels = [i+1 for i in range(8) if (skip_mask >> i) & 1]
        print(f"跳过检查掩码设置: {bin(skip_mask)} (跳过检查通道: {skipped_channels})")
        
    except ValueError:
        print(f"无效的跳过检查掩码格式: {args.skip_mask}")
        print("使用默认值: 0b00000000 (检查所有通道)")
        skip_mask = 0b00000000
    
    app = QApplication(sys.argv)
    
    # 设置应用程序图标和信息
    app.setApplicationName("ESP32示波器上位机")
    app.setApplicationVersion("1.0")
    
    # 创建并显示主窗口
    window = OscilloscopeMainWindow(skip_mask)
    window.show()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
