import rospy
from wrapper_modbus.BaseModbusClient import BaseModbusClient
from ros_modbus_msg.msg import slider
from ros_modbus_msg.msg import operation

def speed_ms(ch_x:list):
    tmp_speed = client._readRegisters(ch_x[1],ch_x[2])
    tmp_ms = client._readRegisters(ch_x[3],ch_x[2])
    return tmp_speed,tmp_ms 


def oput(name,speed,ms):
    print(f"{name}: speed: {speed}  ms:{ms}")


if __name__ == "__main__":
    rospy.init_node("Listen")

    host = '192.168.1.222'
    port = 502

    client = BaseModbusClient(host)
    print("启动 ModBus 客户端")

    ch0 = ['x_1',10120,2,10150]
    ch1 = ['x_2',10220,2,10250]
    ch2 = [ 'y' ,10320,2,10350]
    ch3 = [ 'z' ,10420,2,10450]

    ch0_speed,ch0_ms = speed_ms(ch0)
    ch1_speed,ch1_ms = speed_ms(ch1)
    ch2_speed,ch2_ms = speed_ms(ch2)
    ch3_speed,ch3_ms = speed_ms(ch3)

    oput(ch0[0],ch0_speed,ch0_ms)
    oput(ch1[0],ch1_speed,ch1_ms)
    oput(ch2[0],ch2_speed,ch2_ms)
    oput(ch3[0],ch3_speed,ch3_ms)


    while not rospy.is_shutdown():
        rospy.sleep(10)
        client.multiAxis_StateRead()
