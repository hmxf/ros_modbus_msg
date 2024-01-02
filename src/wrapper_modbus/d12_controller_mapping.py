controller_axis_mapping = {}   # 控制器每个输出通道的每个功能的起始地址的参数映射字典


# 各功能的起始地址的参数映射字典
# “功能名”：[每轴x个寄存器;   读取的寄存器的基地址;   写入的寄存器的基地址;   单双字;  0:只读 1:只写 2:读写]
date_map = {
    "multiAxisStateRead"         : [6, 12000,  None, 2, 0],     # 读保持寄存器状态
    "multiAxisStop"              : [2,  None, 12096, 2, 1],     # 停止
    "multiAxisRelativeMove"      : [2,  None, 12128, 2, 1],     # 相对移动 
    "multiAxisAbsoluteMove"      : [2, 12160, 12160, 2, 2],     # 绝对移动
    "multiAxisRelativeSpeedMove" : [4,  None, 12552, 2, 1],     # 相对移动 + 指定速度
    "multiAxisAbsoluteSpeedMove" : [4,  None, 12616, 2, 1],     # 绝对移动 + 指定速度
    "multiAxisHoming"            : [2,  None, 12680, 2, 1],     # 回零
}

# 计算公式
def address_calculate_formula(key, num_ch):
    """
    每通道的每个功能的其实地址的计算公式
    """
    data = date_map[key]
    if data[4] == 0:
        return data[1] + data[0] * num_ch
    elif data[4] == 1:
        return data[2] + data[0] * num_ch
    else:
        return data[1] + data[0] * num_ch


# 计算函数
def formula_result(num):
    """
    通过date_map和address_caluculate_formula 公式计算每个通道的每个功能的起始地址；
    """
    axis_map = {}
    for num_ch in range(num):
        axis_map[num_ch] = {}
        for key in date_map:
            if key in date_map:
                k = address_calculate_formula(key, num_ch)
                axis_map[num_ch][key] = k
    for num_ch, values in axis_map.items():
        controller_axis_mapping[num_ch] = [values[key] for key in values]


def StateRead_OperationTable(num_elements):
    StateRead_data = []
    for i in range(num_elements):
        # 生成每个操作的名称和索引
        operation_name = f"CH{i})"
        status_index = i * 6 + 1
        position_index = i * 6 + 3
        speed_index = i * 6 + 5
        
        # 创建操作字典并添加到列表中
        operation_data = {
            "name": operation_name,
            "status_index": status_index,
            "position_index": position_index,
            "speed_index": speed_index
        }
        StateRead_data.append(operation_data)

    return StateRead_data


num = 4         # 轴数
formula_result(num)
# StateRead_OperationTable(num)