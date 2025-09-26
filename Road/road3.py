import struct
import serial
import serial.tools.list_ports as list_ports
from collections import deque
import threading
from collections import deque
import numpy as np
import time
from Com import *

# 定义方向常量
DOWN = (1, 0)
UP = (-1, 0)
RIGHT = (0, 1)
LEFT = (0, -1)
DIRECTIONS = [DOWN, UP, RIGHT, LEFT]
DIR_NAMES = {DOWN: 'DOWN', UP: 'UP', RIGHT: 'RIGHT', LEFT: 'LEFT'}

# 定义速度区域
SPEED_ZONE = {"normal": 2.0, "curve": 1.0, "shelf": 0.5, "slow": 0.5}  # 添加slow区域

valid_coords_by_row = {
    0:  [6, 12],
    1:  [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12],
    2:  [6, 12],
    3:  [6, 12],
    4:  list(range(2, 63)),  # 2 到 62 列都有效
    5:  [0, 3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48, 51, 54, 57, 60, 63],
    6:  [0, 3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48, 51, 54, 57, 60, 63],
    7:  [0, 3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48, 51, 54, 57, 60, 63],
    8:  [0, 3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48, 51, 54, 57, 60, 63],
    9:  [0, 3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48, 51, 54, 57, 60, 63],
    10: [0, 3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48, 51, 54, 57, 60, 63, 64, 65, 66],
    11: [0, 3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48, 51, 54, 57, 60, 63],
    12: [0, 3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48, 51, 54, 57, 60, 63],
    13: [0, 3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48, 51, 54, 57, 60, 63],
    14: [0, 3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48, 51, 54, 57, 60, 63],
    15: [63],
    16: [63, 64, 65, 66],
    17: [63],
    18: [63],
    19: [63],
}

# 添加需要过滤的点####################################
filtered_points = set()

# row == 1 and col in range(1, 4)
for col in range(1, 4):
    filtered_points.add((1, col))

# row == 4 and col == 62
filtered_points.add((4, 62))

# row == 1 and col in range(5, 11)
for col in range(5, 11):
    filtered_points.add((1, col))

# row == 4 and col in range(3, 61, 3)
for col in range(3, 61, 3):
    filtered_points.add((4, col))

# row == 5 and col in range(0, 64, 3)
for col in range(0, 64, 3):
    filtered_points.add((5, col))

# row in [7, 9, 10, 11, 13, 15, 16, 17, 19] and col == 63
for row in [7, 9, 10, 11, 13, 15, 16, 17, 19]:
    filtered_points.add((row, 63))

# row in [10, 16] and col == 64
for row in [10, 16]:
    filtered_points.add((row, 64))
# 需要过滤的点######################################

# 有效区域定义  
def is_valid(row, col):
    return row in valid_coords_by_row and col in valid_coords_by_row[row]

# 检查是否允许转向（只能在主干道第2行转向）
def is_turn_allowed(row, col, action, direction):
    # 在货架区（第3-10行）不允许转向来改变列
    if (7 <= row <= 14 and 0 <= col <= 60
         or row == 0 and col == 6 
         or row == 4 and col == 62 
         or row == 3 and col == 12 
         or row in [7, 9, 10, 11, 13, 15, 16, 17, 19] and col == 63
         or row in [10, 16] and col == 64
         or row in [10, 16] and col == 66
         or row == 1 and col in range(0, 4)
         or row == 1 and col in range(5, 11)
         or row == 5 and col in range(0, 64, 3) 
         or row == 4 and col in range(3, 61, 3)):
        if action in ['left_turn', 'right_turn', 'left_back_turn', 'right_back_turn']:
            return False
    if  row == 1 and col == 4:
        if action in ['left_turn', 'right_back_turn']:
            return False
        
    if  (row == 4 and col in range(2, 60, 3) 
         or row == 3 and col == 6 
         or row == 2 and col == 6 
         or row == 6 and col == 63
         or row == 8 and col == 63
         or row == 14 and col == 63
         or row == 2 and col == 12):
        if action in ['right_turn', 'left_back_turn']:
            return False
    
    if  (row == 4 and col in range(4, 63, 3)
         or row == 12 and col == 63
         or row == 18 and col == 63
         ):
        if action in ['left_turn', 'right_back_turn']:
            return False

    if  row == 6 and col in range(0, 61, 3):
        if action in ['left_turn', 'right_back_turn']:
            return False
     
    return True

# def is_area_cut(row, col, action, direction):
#     # 去掉实际路径中没有的二维码坐标
#     if (
#          row == 1 and col in range(1, 4)
#          or row == 4 and col == 62
#          or row == 1 and col in range(5, 11)
#          or row == 4 and col in range(3, 61, 3)
#          or row == 5 and col in range(0, 64, 3)
#          or row in [7, 9, 10, 11, 13, 15, 16, 17, 19] and col == 63
#          or row in [10, 16] and col == 64):
        
#             return False
    
#     return True

# 转向函数
def turn_left(row, col, direction):
    if direction == DOWN:
        new_dir = RIGHT
        new_row, new_col = row + 2, col + 2
    elif direction == UP:
        new_dir = LEFT
        new_row, new_col = row - 2, col - 2
    elif direction == RIGHT:
        new_dir = UP
        new_row, new_col = row - 2, col + 2
    elif direction == LEFT:
        new_dir = DOWN
        new_row, new_col = row + 2, col - 2
    return (new_row, new_col, new_dir)

def turn_right(row, col, direction):
    if direction == DOWN:
        new_dir = LEFT
        new_row, new_col = row + 2, col - 2
    elif direction == UP:
        new_dir = RIGHT
        new_row, new_col = row - 2, col + 2
    elif direction == RIGHT:
        new_dir = DOWN
        new_row, new_col = row + 2, col + 2
    elif direction == LEFT:
        new_dir = UP
        new_row, new_col = row - 2, col - 2
    return (new_row, new_col, new_dir)

def turn_back_left(row, col, direction):
    if direction == DOWN:
        new_dir = LEFT
        new_row, new_col = row - 2, col + 2
    elif direction == UP:
        new_dir = RIGHT
        new_row, new_col = row + 2, col - 2
    elif direction == RIGHT:
        new_dir = DOWN
        new_row, new_col = row - 2, col - 2
    elif direction == LEFT:
        new_dir = UP
        new_row, new_col = row + 2, col + 2
    return (new_row, new_col, new_dir)

def turn_back_right(row, col, direction):
    if direction == DOWN:
        new_dir = RIGHT
        new_row, new_col = row - 2, col - 2
    elif direction == UP:
        new_dir = LEFT
        new_row, new_col = row + 2, col + 2
    elif direction == RIGHT:
        new_dir = UP
        new_row, new_col = row + 2, col - 2
    elif direction == LEFT:
        new_dir = DOWN
        new_row, new_col = row - 2, col + 2
    return (new_row, new_col, new_dir)

# 前进和后退
def move_forward(row, col, direction):
    dr, dc = direction
    new_row, new_col = row + dr, col + dc
    return (new_row, new_col, direction)

def move_backward(row, col, direction):
    dr, dc = direction
    new_row, new_col = row - dr, col - dc
    return (new_row, new_col, direction)

# 获取所有可能的移动（考虑转向限制）
def get_moves(state):
    row, col, dir = state
    moves = []
    
    # 前进
    new_state = move_forward(row, col, dir)
    if is_valid(new_state[0], new_state[1]):
        moves.append(('forward', new_state))
    
    # 后退
    new_state = move_backward(row, col, dir)
    if is_valid(new_state[0], new_state[1]):
        moves.append(('backward', new_state))
    
    # 检查转向操作是否允许
    if is_turn_allowed(row, col, 'left_turn', dir):
        new_state = turn_left(row, col, dir)
        if is_valid(new_state[0], new_state[1]):
            moves.append(('left_turn', new_state))
    
    if is_turn_allowed(row, col, 'right_turn', dir):
        new_state = turn_right(row, col, dir)
        if is_valid(new_state[0], new_state[1]):
            moves.append(('right_turn', new_state))
    
    if is_turn_allowed(row, col, 'left_back_turn', dir):
        new_state = turn_back_left(row, col, dir)
        if is_valid(new_state[0], new_state[1]):
            moves.append(('left_back_turn', new_state))
    
    if is_turn_allowed(row, col, 'right_back_turn', dir):
        new_state = turn_back_right(row, col, dir)
        if is_valid(new_state[0], new_state[1]):
            moves.append(('right_back_turn', new_state))
    
    return moves

# BFS路径规划
def bfs_segment(start, goal):
    """
    独立的BFS路径段搜索，每个段有自己的visited集合
    """
    queue = deque([(start, [])])
    visited = set()
    visited.add(start)
    
    while queue:
        state, path = queue.popleft()
        
        # 检查是否到达目标状态
        if state[:2] == goal[:2] and state[2] == goal[2]:
            return path
        
        # 获取所有可能的移动
        for action, new_state in get_moves(state):
            if new_state not in visited:
                visited.add(new_state)
                queue.append((new_state, path + [(action, new_state)]))
    print("path:", path)
    
    return None
    
# 第二阶段特殊路径规划
def special_path_to_restock(start, goal):
    """
    特殊路径规划：从取货点到补货点
    要求：只能后退到第6行，然后只能做右后退到第4行，更新车头朝向后，再继续做BFS
    如果特殊路径规划失败，直接返回None，不进行普通BFS
    """
    row, col, direction = start
    print("row, col, direction", row, col, direction)
    # 检查是否已经在目标位置
    if (row, col) == goal[:2] and direction == goal[2]:
        return []
    
    # 第一步：后退到第3行
    if row > 6:
        # 计算需要后退的步数
        steps_to_row3 = row - 6
        path = []
        current_state = start
        
        # 后退到第3行
        for _ in range(steps_to_row3):
            new_state = move_backward(*current_state)
            if not is_valid(new_state[0], new_state[1]):
                print("后退到第3行失败！")
                return None
            path.append(('backward', new_state))
            current_state = new_state
        
        # 第二步：右后退到第2行（例如从(3,9)车头朝下退到(2,8)车头朝右）
        if current_state[0] == 6 and current_state[2] == DOWN:
            if col ==0 or col == 3:
                # 执行后退转向
                new_state = turn_back_left(*current_state)
                if not is_valid(new_state[0], new_state[1]):
                    print("右后退到第2行失败！")
                    return None
                path.append(('left_back_turn', new_state))
                current_state = new_state
                
                # 第三步：使用BFS规划剩余路径
                remaining_path = bfs_segment(current_state, goal)
                if not remaining_path:
                    print("BFS规划剩余路径失败！")
                    return None
                return path + remaining_path

            else:
                # 执行右后退转向
                new_state = turn_back_right(*current_state)
                if not is_valid(new_state[0], new_state[1]):
                    print("右后退到第2行失败！")
                    return None
                path.append(('right_back_turn', new_state))
                current_state = new_state
                
                # 第三步：使用BFS规划剩余路径
                remaining_path = bfs_segment(current_state, goal)
                if not remaining_path:
                    print("BFS规划剩余路径失败！")
                    return None
                return path + remaining_path
        else:
            print(f"当前位置{current_state}不符合右后退条件！")
            return None
    else:
        print("起始位置不在第3行以上，无法进行特殊路径规划！")
        return None

# 获取速度  
def get_speed_zone(row, col, action, is_special_point=False):
    # 如果是转向操作，使用慢速
    if action in ['left_turn', 'right_turn', 'left_back_turn', 'right_back_turn']:
        return "slow", SPEED_ZONE["slow"]
    
    # 如果是特殊点（段首尾点），使用慢速
    if is_special_point:
        return "slow", SPEED_ZONE["slow"]
    # 默认情况返回正常速度
    return "normal speed", SPEED_ZONE["normal"]

# 规划出入库任务
def plan_restock_task(init_pos, init_direction, pick_pos, empty_col, task_type):
    # 初始位置和方向
    start = (init_pos[0], init_pos[1], init_direction)
    
    # 取货位置，车头需要朝向货架
    pick_goal_direction = DOWN  # 货架区都需要朝下取货
    pick_goal = (pick_pos[0], pick_pos[1], pick_goal_direction)

    if task_type == 1:
        # 补货位置 (6,19)，需要朝右
        restock_pos = (10, 66)
        restock_goal = (restock_pos[0], restock_pos[1], RIGHT)
    else:
        # 出库位置 (1,0)，需要朝左
        restock_pos = (1, 0)
        restock_goal = (restock_pos[0], restock_pos[1], LEFT)
    
    # 空闲列位置，放在第5行，需要朝下
    empty_pos = empty_col
    empty_goal = (empty_pos[0], empty_pos[1], DOWN)
    
    # 路径规划
    print(f"规划从 {start} 到 {pick_goal} 的路径...")
    path1 = bfs_segment(start, pick_goal)  # 去取货
    
    if not path1:
        print("无法找到取货路径！")
        return None
    
    path1 = bfs_segment(start, pick_goal)
    # print("path1: ", path1)
    if not path1:
        print("第一阶段路径规划失败！")
        return None
    
    if task_type == 1:
        print("规划第二阶段：取货点 → 补货点") 
    else:
        print("规划第二阶段：取货点 → 出库点") 
    path2 = special_path_to_restock(pick_goal, restock_goal)
    # print("path2: ", path2)
    if not path2:
        print("第二阶段路径规划失败！")
        return None
    
    if task_type == 1:
        print("规划第三阶段：补货点 → 放货点")
    else:
        print("规划第三阶段：出库点 → 放货点") 
    path3 = bfs_segment(restock_goal, empty_goal)
    # print("path3: ", path3)
    if not path3:
        print("第三阶段路径规划失败！")
        return None
    
    # 合并路径
    full_path = path1 + path2 + path3
    
    # 生成详细路径和速度
    detailed_path = []
    current_state = start
    
    # 标记特殊点（段首尾点）
    special_points = set()
    if path1:
        special_points.add((path1[-1][1][0], path1[-1][1][1]))  # 第一段最后一个点
    if path2:
        special_points.add((path2[0][1][0], path2[0][1][1]))    # 第二段第一个点
        special_points.add((path2[-1][1][0], path2[-1][1][1]))  # 第二段最后一个点
    if path3:
        special_points.add((path3[0][1][0], path3[0][1][1]))    # 第三段第一个点
        special_points.add((path3[-1][1][0], path3[-1][1][1]))  # 第三段最后一个点
    
    for i, (action, state) in enumerate(full_path):
        pos = (current_state[0], current_state[1])
        is_special_point = pos in special_points
        zone, speed = get_speed_zone(pos[0], pos[1], action, is_special_point)
        detailed_path.append({
            'position': pos,
            'direction': current_state[2],
            'direction_name': DIR_NAMES[current_state[2]],
            'action': action,
            'speed': speed,
            'speed_zone': zone
        })
        current_state = state
    
    # 添加最终停止状态
    detailed_path.append({
        'position': current_state[:2],
        'direction': current_state[2],
        'direction_name': DIR_NAMES[current_state[2]],
        'action': 'stop',
        'speed': 0.0,
        'speed_zone': 'stop'
    })
    
    return detailed_path

class AGVCommunication:
    def __init__(self, port=None, baudrate=115200):
        self.ser = None
        self.running = False
        self.receive_thread = None
        self.port = port
        self.baudrate = baudrate
        self.task_callback = None
        
    def get_available_ports(self):
        """获取所有可用串口"""
        ports = list_ports.comports()
        return [port.device for port in ports]
    
    def connect(self, port=None, baudrate=None):
        """连接串口"""
        if port:
            self.port = port
        if baudrate:
            self.baudrate = baudrate
            
        if not self.port:
            available_ports = self.get_available_ports()
            if not available_ports:
                print("没有找到可用的串口设备！")
                return False
            self.port = available_ports[0]
            print(f"自动选择串口: {self.port}")
        
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"成功连接到串口: {self.port}")
            return True
        except Exception as e:
            print(f"连接串口失败: {e}")
            return False
    
    def disconnect(self):
        """断开串口连接"""
        self.stop()
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("串口已关闭")
    
    def parse_task_data(self, data):
        """解析从控制端接收的任务数据"""
        # 协议格式：任务类型(1B) + 初始行(1B) + 初始列(1B) + 初始方向(1B) + 
        #           取货行(1B) + 取货列(1B) + 空闲列行(1B) + 空闲列列(1B)
        if len(data) < 8:
            print(f"数据长度不足: {len(data)}字节")
            return None
        
        task_type = data[0]
        init_pos = (data[1], data[2])  # (行, 列)
        init_direction_val = data[3]
        pick_pos = (data[4], data[5])  # (行, 列)
        empty_col = (data[6], data[7])  # (行, 列)
        
        # 方向值映射到方向常量
        direction_map = {0: DOWN, 1: UP, 2: RIGHT, 3: LEFT}
        init_direction = direction_map.get(init_direction_val, DOWN)
        
        task_info = {
            'task_type': task_type,
            'init_pos': init_pos,
            'init_direction': init_direction,
            'pick_pos': pick_pos,
            'empty_col': empty_col
        }
        
        print(f"解析到任务数据: 类型={task_type}, 初始位置={init_pos}, "
              f"初始方向={DIR_NAMES[init_direction]}, 取货位置={pick_pos}, "
              f"空闲列={empty_col}")
        
        return task_info
    
    def encode_path_data(self, detailed_path):
        """将详细路径编码为传输格式"""
        if not detailed_path:
            return None
        
        # 动作映射表
        action_map = {
            'forward': 0x01,
            'backward': 0x02,
            'left_turn': 0x03,
            'right_turn': 0x04,
            'left_back_turn': 0x05,
            'right_back_turn': 0x06,
            'stop': 0x00
        }
        
        # 构建路径点数据
        path_data = bytearray()
        for step in detailed_path:
            row, col = step['position']
            action_code = action_map.get(step['action'], 0x00)
            speed_int = int(step['speed'] * 10)  # 速度放大10倍转为整数
            
            path_data.extend([row, col, action_code, speed_int])
        
        # 构建数据帧
        command = 0x01  # 路径数据命令
        data_length = len(path_data)
        length_high = (data_length >> 8) & 0xFF
        length_low = data_length & 0xFF
        
        # 构建不带CRC的数据
        sdata = [command, length_low, length_high] + list(path_data)
        fmt = '>' + 'B' * len(sdata)
        crc_byte = struct.pack(fmt, *sdata)
        
        # 计算CRC
        crc = CRC8(crc_byte)
        
        # 添加CRC并打包
        sdata.append(crc)
        fmt_final = '>' + 'B' * len(sdata)
        final_data = struct.pack(fmt_final, *sdata)
        
        # 添加帧头
        frame_header = bytes([0xAB, 0xBA])
        return frame_header + final_data
    
    def send_path(self, detailed_path):
        """发送完整路径数据到控制端"""
        if not self.ser or not self.ser.is_open:
            print("串口未连接")
            return False
        
        if not detailed_path:
            print("无有效路径可发送")
            return False
        
        try:
            path_data = self.encode_path_data(detailed_path)
            if not path_data:
                print("路径数据编码失败")
                return False
            
            self.ser.write(path_data)
            print(f"已发送路径数据，长度: {len(path_data)}字节")
            print(f"数据内容: {path_data.hex(' ')}")
            return True
            
        except Exception as e:
            print(f"发送数据失败: {e}")
            return False
    
    def _receive_thread_func(self):
        """接收数据线程函数"""
        buffer = bytearray()
        while self.running and self.ser and self.ser.is_open:
            # try:
                # 读取可用数据
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    buffer.extend(data)
                    
                    # 打印接收到的原始数据
                    print(f"[接收原始数据] 长度: {len(data)}字节, 内容: {data.hex(' ').upper()}")
                    print(f"[缓冲区状态] 当前长度: {len(buffer)}字节")

                    # 查找帧头
                    while len(buffer) >= 2:
                        if buffer[0] == 0xAB and buffer[1] == 0xBA:
                            # 找到帧头，尝试解析完整帧
                            if len(buffer) >= 5:  # 至少要有命令+长度
                                command = buffer[2]
                                data_length = (buffer[4] << 8) | buffer[3]
                                total_length = 5 + data_length + 1  # 头2+命令1+长度2+数据+CRC1
                                
                                if len(buffer) >= total_length:
                                    # 提取完整帧
                                    frame = bytes(buffer[:total_length])
                                    buffer = buffer[total_length:]
                                    
                                    # 验证CRC
                                    crc_received = frame[-1]
                                    crc_data = frame[:-1]  # 从命令字节到数据末尾
                                    # crc_data = frame  # 从命令字节到数据末尾
                                    crc_calculated = CRC8(crc_data)
                                    
                                    print(f"CRC计算数据: {crc_data.hex(' ').upper()}")
                                    print(f"接收到的CRC: 0x{crc_received:02X}")
                                    print(f"计算出的CRC: 0x{crc_calculated:02X}")
                                    
                                    if crc_received == crc_calculated:
                                        # CRC校验通过，解析任务数据
                                        task_data = self.parse_task_data(frame[5:-1])  # 跳过帧头、命令、长度
                                        if task_data and self.task_callback:
                                            self.task_callback(task_data, self)  # 传递self实例
                                    else:
                                        print("CRC校验失败，丢弃数据帧")
                                else:
                                    break  # 数据不完整，等待更多数据
                            else:
                                break  # 数据不完整，等待更多数据
                        else:
                            # 不是帧头，丢弃第一个字节继续查找
                            buffer.pop(0)
                
                time.sleep(0.01)
                
            # except Exception as e:
            #     print(f"接收数据错误: {e}")
            #     time.sleep(0.1)
    
    def start(self, task_callback=None):
        """启动通信"""
        if not self.ser or not self.ser.is_open:
            if not self.connect():
                return False
        
        self.task_callback = task_callback
        self.running = True
        self.receive_thread = threading.Thread(target=self._receive_thread_func)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        
        print("AGV通信已启动，等待任务数据...")
        return True
    
    def stop(self):
        """停止通信"""
        self.running = False
        if self.receive_thread:
            self.receive_thread.join(timeout=1.0)
        self.disconnect()
        print("AGV通信已停止")

# 任务回调函数
def task_callback(task_data, agv_instance):
    """处理接收到的任务数据"""
    print("=" * 50)
    print("收到控制端下发的任务数据:")
    print(f"任务类型: {'入库' if task_data['task_type'] == 1 else '出库'}")
    print(f"起始位置: {task_data['init_pos']}")
    print(f"起始方向: {DIR_NAMES[task_data['init_direction']]}")
    print(f"取货位置: {task_data['pick_pos']}")
    print(f"空闲列位置: {task_data['empty_col']}")
    print("=" * 50)
    
    # 使用控制端下发的参数进行路径规划
    # 真实通信中规划出的路径
    detailed_path = plan_restock_task(
        task_data['init_pos'],
        task_data['init_direction'],
        task_data['pick_pos'],
        task_data['empty_col'],
        task_data['task_type']
    )
    # print("Before cut")
    # for i, step in enumerate(detailed_path):
    #         print(f"步骤 {i+1}: 位置{step['position']}, 方向:{step['direction_name']}, "
    #               f"动作:{step['action']}, 速度:{step['speed']}m/s, 区域:{step['speed_zone']}")
    # print("After cut")
    if detailed_path:
        detailed_path = [step for step in detailed_path 
                    if step['position'] not in filtered_points]
        print("\n详细路径规划结果:")
        for i, step in enumerate(detailed_path):
            print(f"步骤 {i+1}: 位置{step['position']}, 方向:{step['direction_name']}, "
                  f"动作:{step['action']}, 速度:{step['speed']}m/s, 区域:{step['speed_zone']}")
        
        # 发送路径数据回控制端
        if agv_instance.send_path(detailed_path):
            print("路径数据已成功发送到控制端")
        else:
            print("路径数据发送失败")
    else:
        print("路径规划失败，无法生成有效路径")
        # 可以发送错误信息回控制端

def main():
    """主函数"""
    # agv_com = AGVCommunication()
    
    # # 获取可用串口
    # available_ports = agv_com.get_available_ports()
    # if not available_ports:
    #     print("没有找到可用的串口设备！")
    #     return
    
    # print("可用串口:", available_ports)

    
    # # 选择串口
    # port_choice = input("请输入串口编号(直接回车使用第一个): ")
    # if port_choice.isdigit() and int(port_choice) < len(available_ports):
    #     port = available_ports[int(port_choice)]
    # else:
    #     port = available_ports[0]
    
    port_name = "COM2"  # 这里填写您想要的串口
    baud_rate = 115200
    
    agv_com = AGVCommunication(port=port_name, baudrate=baud_rate)
    
    # 打印设置的串口值
    print(f"设置的串口: port=\"{port_name}\"")
    print(f"设置的波特率: baudrate={baud_rate}")
    
    # 启动通信
    if agv_com.start(task_callback):
        try:
            print("AGV系统已启动，等待控制端下发任务...")
            print("按Ctrl+C退出程序")
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("程序退出")
        finally:
            agv_com.stop()
    else:
        print("启动AGV通信失败")

if __name__ == "__main__":
    main()