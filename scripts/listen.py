import rospy
from wrapper_modbus.BaseModbusClient import BaseModbusClient

def speed_ms(ch_x:list):
    tmp_speed = client._readRegisters(ch_x[1],ch_x[2])
    tmp_ms = client._readRegisters(ch_x[3],ch_x[2])
    print(f"{ch_x[0]}: speed: {tmp_speed}  ms:{tmp_ms}")


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

    speed_ms(ch0)
    speed_ms(ch1)
    speed_ms(ch2)
    speed_ms(ch3)


    while not rospy.is_shutdown():
        rospy.sleep(1)
        result = client.multiAxis_StateRead()
