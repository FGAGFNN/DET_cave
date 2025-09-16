from collections import deque
import numpy as np

# 定义方向常量
DOWN = (1, 0)
UP = (-1, 0)
RIGHT = (0, 1)
LEFT = (0, -1)
DIRECTIONS = [DOWN, UP, RIGHT, LEFT]
DIR_NAMES = {DOWN: 'DOWN', UP: 'UP', RIGHT: 'RIGHT', LEFT: 'LEFT'}

# 定义速度区域
SPEED_ZONE = {"main": 2.0, "curve": 1.0, "shelf": 0.5}

# 有效区域定义  
def is_valid(row, col):
    if (row, col) in [(1,0), (1,1), (1,2), (1,3), (1,5), (0,3), (0,5), (2,16), (3,17), (4,17), (5,17), (6,17), (7,17), (4,18), (6,18), (8,18), (4,19), (6,19), (8,19)]:
        return True
    if 2 <= row <= 10 and 2 <= col <= 15:
        if row == 2 and col ==2:
            return False
        else:
            return True
    
    return False

# 检查是否允许转向（只能在主干道第2行转向）
def is_turn_allowed(row, col, action):
    # 在货架区（第3-10行）不允许转向来改变列
    if row >= 3 and row <= 10 and 2 <= col <= 15 or row == 0 and col == 3:
        if action in ['left_turn', 'right_turn', 'left_back_turn', 'right_back_turn']:
            return False
    if  row == 1 and col == 2 or row == 2 and col == 4:
        if action in ['left_turn', 'right_back_turn']:
            return False
    if  row == 1 and col == 5:
        if action in ['right_turn', 'left_back_turn']:
            return 
    return True

# 转向函数
def turn_left(row, col, direction):
    if direction == DOWN:
        new_dir = RIGHT
        new_row, new_col = row + 1, col + 1
    elif direction == UP:
        new_dir = LEFT
        new_row, new_col = row - 1, col - 1
    elif direction == RIGHT:
        new_dir = UP
        new_row, new_col = row - 1, col + 1
    elif direction == LEFT:
        new_dir = DOWN
        new_row, new_col = row + 1, col - 1
    return (new_row, new_col, new_dir)

def turn_right(row, col, direction):
    if direction == DOWN:
        new_dir = LEFT
        new_row, new_col = row + 1, col - 1
    elif direction == UP:
        new_dir = RIGHT
        new_row, new_col = row - 1, col + 1
    elif direction == RIGHT:
        new_dir = DOWN
        new_row, new_col = row + 1, col + 1
    elif direction == LEFT:
        new_dir = UP
        new_row, new_col = row - 1, col - 1
    return (new_row, new_col, new_dir)

def turn_back_left(row, col, direction):
    if direction == DOWN:
        new_dir = LEFT
        new_row, new_col = row - 1, col + 1
    elif direction == UP:
        new_dir = RIGHT
        new_row, new_col = row + 1, col - 1
    elif direction == RIGHT:
        new_dir = DOWN
        new_row, new_col = row - 1, col - 1
    elif direction == LEFT:
        new_dir = UP
        new_row, new_col = row + 1, col + 1
    return (new_row, new_col, new_dir)

def turn_back_right(row, col, direction):
    if direction == DOWN:
        new_dir = RIGHT
        new_row, new_col = row - 1, col - 1
    elif direction == UP:
        new_dir = LEFT
        new_row, new_col = row + 1, col + 1
    elif direction == RIGHT:
        new_dir = UP
        new_row, new_col = row + 1, col - 1
    elif direction == LEFT:
        new_dir = DOWN
        new_row, new_col = row - 1, col + 1
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
    if is_turn_allowed(row, col, 'left_turn'):
        new_state = turn_left(row, col, dir)
        if is_valid(new_state[0], new_state[1]):
            moves.append(('left_turn', new_state))
    
    if is_turn_allowed(row, col, 'right_turn'):
        new_state = turn_right(row, col, dir)
        if is_valid(new_state[0], new_state[1]):
            moves.append(('right_turn', new_state))
    
    if is_turn_allowed(row, col, 'left_back_turn'):
        new_state = turn_back_left(row, col, dir)
        if is_valid(new_state[0], new_state[1]):
            moves.append(('left_back_turn', new_state))
    
    if is_turn_allowed(row, col, 'right_back_turn'):
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
    要求：只能后退到第3行，然后只能做右后退到第2行，更新车头朝向后，再继续做BFS
    如果特殊路径规划失败，直接返回None，不进行普通BFS
    """
    row, col, direction = start
    print("row, col, direction", row, col, direction)
    # 检查是否已经在目标位置
    if (row, col) == goal[:2] and direction == goal[2]:
        return []
    
    # 第一步：后退到第3行
    if row > 3:
        # 计算需要后退的步数
        steps_to_row3 = row - 3
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
        if current_state[0] == 3 and current_state[2] == DOWN:
            if col ==2 or col == 3:
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
def get_speed_zone(row, col):
    if row == 2:
        return "main"
    elif 5 <= row <= 10 and 2 <= col <= 15:
        return "shelf"
    else:
        return "curve"

# 规划出入库任务
def plan_restock_task(init_pos, init_direction, pick_pos, empty_col, task_type):
    # 初始位置和方向
    start = (init_pos[0], init_pos[1], init_direction)
    
    # 取货位置，车头需要朝向货架
    pick_goal_direction = DOWN  # 货架区都需要朝下取货
    pick_goal = (pick_pos[0], pick_pos[1], pick_goal_direction)

    if task_type == 1:
        # 补货位置 (6,19)，需要朝右
        restock_pos = (6, 19)
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
    print("path1: ", path1)
    if not path1:
        print("第一阶段路径规划失败！")
        return None
    
    if task_type == 1:
        print("规划第二阶段：取货点 → 补货点") 
    else:
        print("规划第二阶段：取货点 → 出库点") 
    path2 = special_path_to_restock(pick_goal, restock_goal)
    print("path2: ", path2)
    if not path2:
        print("第二阶段路径规划失败！")
        return None
    
    if task_type == 1:
        print("规划第三阶段：补货点 → 放货点")
    else:
        print("规划第三阶段：出库点 → 放货点") 
    path3 = bfs_segment(restock_goal, empty_goal)
    print("path3: ", path3)
    if not path3:
        print("第三阶段路径规划失败！")
        return None
    
    # 合并路径
    full_path = path1 + path2 + path3
    
    
    # 生成详细路径和速度
    detailed_path = []
    current_state = start
    for action, state in full_path:
        pos = (current_state[0], current_state[1])
        zone = get_speed_zone(pos[0], pos[1])
        speed = SPEED_ZONE[zone]
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

# 示例使用 - 初始车头朝下
order_command = 2 # 任务类别：出库、入库
task_type = 1 # 默认任务为入库
if order_command == 2:
    task_type = order_command
init_pos = (2, 9) # 叉车起始位置
init_direction = LEFT
pick_pos = (5, 3) # 目标取货位置
empty_col = (5, 7) # 取货区空闲列

if task_type == 1:
    print("开始规划入库路径...")
else:
    print("开始规划出库路径...")
path = plan_restock_task(init_pos, init_direction, pick_pos, empty_col, task_type)

if path:
    print("\n详细路径规划:")
    for i, step in enumerate(path):
        print(f"步骤 {i+1}: 位置{step['position']}, 方向:{step['direction_name']}, "
              f"动作:{step['action']}, 速度:{step['speed']}m/s, 区域:{step['speed_zone']}")
else:
    print("路径规划失败！")