import rospy
from wrapper_modbus.BaseModbusClient import BaseModbusClient
from ros_modbus_msg.msg import slider
from ros_modbus_msg.msg import operation
import random


def set_spms(client):
    ch0 = ['x_1',10120,2,10150]
    ch1 = ['x_2',10220,2,10250]
    ch2 = ['y',10320,2,10350]

    client._writeRegisters(ch0[1],[0,30000])
    client._writeRegisters(ch0[3],[0,3000])

    client._writeRegisters(ch1[1],[0,30000])
    client._writeRegisters(ch1[3],[0,3000])

    client._writeRegisters(ch2[1],[0,30000])
    client._writeRegisters(ch2[3],[0,3000])

def start_client():
    host = '192.168.1.222'
    port = 502

    client = BaseModbusClient(host)
    print("启动 ModBus 客户端")
    
    return client


def test01_Abs_move():
    pub =rospy.Publisher("sub_param",slider,queue_size=50) 
    complete = 1

    slider_msg = slider()
    slider_msg.operation = 1
    while not rospy.is_shutdown():
        if  complete:
            slider_msg.A_x = random.randint(1,200)*100
            slider_msg.A_y = random.randint(1,200)*100
            slider_msg.A_z = random.randint(1,200)*100
            pub.publish(slider_msg)
            complete = 0
            rospy.loginfo("A_x:%d A_y:%d A_z:%d",slider_msg.A_x,slider_msg.A_y,slider_msg.A_z)
        else:
            continue

if __name__ == "__main__":
    rospy.init_node("Test")


    # client = start_client()
    # set_spms(client)
    test01_Abs_move()
    