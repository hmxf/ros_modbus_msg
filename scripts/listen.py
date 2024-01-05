import rospy
from wrapper_modbus.d12_modbus_client import D12ModbusClient
from wrapper_modbus.d12_controller_mapping import *


def SpeedandMs(ch_x:list):
    tmp_speed = client._readRegisters(ch_x[1],ch_x[2])
    tmp_ms = client._readRegisters(ch_x[3],ch_x[2])
    rospy.loginfo(f"{ch_x[0]}: speed: {tmp_speed[1]}  ms:{tmp_ms[1]}")

def Monitor_X_Axis_Position(result):
    """
    Monitor whether the position of the two x-axes is consistent, 
    (And the difference between the two axes is 5 units to meet the requirements
    """
    rospy.loginfo(f"Monitor_X_Axis_Position:{result}")
    if abs(result[3] - result[9]) not in range(0, 5):
        rospy.logwarn(f"Warning: The x-axis positions are not synchronized! Value is {abs(result[3] - result[9])}")
        client.multiAxis_EMERGENCYSTOP(multiAxis_EMERGENCYSTOP_data[0],multiAxis_EMERGENCYSTOP_data[1])

if __name__ == "__main__":
    rospy.init_node("Listen")

    host = '192.168.1.222'

    client = D12ModbusClient(host)
    rospy.loginfo("启动 ModBus 客户端")



    address_read_start = controller_axis_mapping[0][0]
    num_registers = date_map["multiAxisStateRead"][0]*num_ch

    # 对各通道的速度相关值输出一次：
    for chx in ch_mapping:
        SpeedandMs(ch_mapping[chx])
    
    # 监听
    while not rospy.is_shutdown():
        rospy.sleep(1)
        result = client.multiAxis_StateRead(address_read_start,num_registers)
        Monitor_X_Axis_Position(result)
