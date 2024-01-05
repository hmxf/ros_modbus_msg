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
def formula_result(num_ch):
    """
    通过date_map和address_caluculate_formula 公式计算每个通道的每个功能的起始地址；
    """
    axis_map = {}
    for ch in range(num_ch):
        axis_map[ch] = {}
        for key in date_map:
            if key in date_map:
                k = address_calculate_formula(key, ch)
                axis_map[ch][key] = k
    for ch, values in axis_map.items():
        controller_axis_mapping[ch] = [values[key] for key in values]


# 读寄存器状态：生成读状态结果索引表
def StateRead_OperationTable(num_elements):
    StateRead_data = []
    for i in range(num_elements):
        # 生成每个操作的名称和索引
        operation_name = f"CH{i}"
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

host                         = '192.168.1.222'
ch0                          = ['x_1',10120,2,10150]
ch1                          = ['x_2',10220,2,10250]
multiAxis_EMERGENCYSTOP_data = [12999,255]
multiAxis_OriginAll_data     = [12997,255]
num_ch = 4         # 通道数
formula_result(num_ch)


#ROS_Modbus: 操作映射表
oper_map = {
    1:"multiAxisStateRead",             # 读保持寄存器状态
    2:"multiAxisStop",                  # 停止
    3:"multiAxisRelativeMove",          # 相对移动 
    4:"multiAxisAbsoluteMove",          # 绝对移动
    5:"multiAxisRelativeSpeedMove",     # 相对移动 + 指定速度
    6:"multiAxisAbsoluteSpeedMove",     # 绝对移动 + 指定速度
    7:"multiAxisHoming",                # 回零点 
}


# d12_modbus_client: 写入数据后需要起始地址和寄存器数量
check_complete_map = {
    0: [10102, 2],
    1: [10302, 2],
    2: [10402, 2]
}

# 各通道的速度 + 微步细分的地址：
ch_mapping = {
    "ch0" : ['X_1', 10120,2,10150],
    "ch1" : ['X_2', 10220,2,10250],
    "ch2" : [' Y ', 10320,2,10350],
    "ch3" : [' Z ', 10420,2,10450]
}
