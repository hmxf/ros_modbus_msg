import rospy
from wrapper_modbus.BaseModbusClient import BaseModbusClient

def SpeedandMs(ch_x:list):
    tmp_speed = client._readRegisters(ch_x[1],ch_x[2])
    tmp_ms = client._readRegisters(ch_x[3],ch_x[2])
    print(f"{ch_x[0]}: speed: {tmp_speed}  ms:{tmp_ms}")

def Monitor_X_Axis_Position(result):
    """
    Monitor whether the position of the two x-axes is consistent, 
    (And the difference between the two axes is 5 units to meet the requirements
    """
    print(result)
    if abs(result[3] - result[9]) not in (0, 5):
        rospy.logwarn(f"Warning: The x-axis positions are not synchronized! Value is {abs(result[3] - result[9])}")
        client.multiAxis_EMERGENCYSTOP()

if __name__ == "__main__":
    rospy.init_node("Listen")

    host = '192.168.1.222'
    port = 502

    client = BaseModbusClient(host)
    print("启动 ModBus 客户端")

    ch0 = ['X_1', 10120,2,10150]
    ch1 = ['X_2', 10220,2,10250]
    ch2 = [' Y ', 10320,2,10350]
    ch3 = [' Z ', 10420,2,10450]

    for chx in [ch0,ch1,ch2,ch3]:
        SpeedandMs(chx)

    while not rospy.is_shutdown():
        rospy.sleep(1)
        result = client.multiAxis_StateRead()
        Monitor_X_Axis_Position(result)
