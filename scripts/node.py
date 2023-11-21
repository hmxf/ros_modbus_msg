import rospy
from wrapper_modbus.BaseModbusClient import BaseModbusClient
from ros_modbus_msg.msg import slider
from ros_modbus_msg.msg import operation
import sys

def speed_ms(ch_x:list):
    tmp_speed = client._readRegisters(ch_x[1],ch_x[2])
    tmp_ms = client._readRegisters(ch_x[3],ch_x[2])
    return tmp_speed,tmp_ms 


def callback_sub_param(msg):
    operation = msg.operation
    A_x = msg.A_x
    x_speed = msg.x_speed
    A_y = msg.A_y
    y_speed = msg.y_speed
    A_z = msg.A_z
    z_speed = msg.z_speed
    if x_speed != 0:
        rospy.loginfo("Received Subscriber Message: Operation:%d  A_x:%d x_speed:%d  A_y:%d y_speed:%d  A_z:%d z_speed:%d", operation,A_x,x_speed,A_y,y_speed,A_z,z_speed)   
    else:
        rospy.loginfo("Received Subscriber Message: Operation:%d  A_x:%d A_y:%d A_z:%d", operation,A_x,A_y,A_z)

    values = [A_x,A_y,A_z] # [x,y,z]
    # multiAxis_AbsoluteMove
    if operation == 1:
        # for v in values:
        #     if v<0 or v > 20200:
        #         raise Exception("Wrong move pos as ", values)
        client.multiAxis_AbsoluteMove(values=values)
    
    # multiAxis Stop
    elif operation == 2:
        client.multiAxis_Stop(values=values)
    
    # multiAxis_Origin
    elif operation == 3:
        values = [x_speed,y_speed,z_speed] # [x,y,z]
        client.multiAxis_Origin(values=values)

    # multiAxis_AbsRunSpeed
    elif operation == 4:
        values = [A_x,x_speed,A_y,y_speed,A_z,z_speed] # [x,y,z]
        client.multiAxis_AbsRunSpeed(values=values)
        
    
def callback_sub_Noparam(msg):
    oper = msg.operation
    rospy.loginfo("Received Subscriber Message: Operation:%d ",oper)

    # multiAxis_StateRead
    if oper == 1:
        client.multiAxis_StateRead()
    
    # multiAxis_EMERGENCYSTOP  所有轴的急停
    elif oper == 2:
        client.multiAxis_EMERGENCYSTOP()

    # multiAxis_Origin_all
    elif oper == 3:
        client.multiAxis_Origin_all()


if __name__ == '__main__':
    rospy.init_node('modbus_subscriber_client')

    host = '192.168.1.222'
    port = 502
    ch0 = ['x_1',10120,2,10150]
    ch1 = ['x_2',10220,2,10250]

    client = BaseModbusClient(host)
    print("启动 ModBus 客户端")

    
    ch0_speed,ch0_ms = speed_ms(ch0)
    ch1_speed,ch1_ms = speed_ms(ch1)

    if (ch0_speed!= ch1_speed) or (ch0_ms != ch1_ms ):
        rospy.logwarn(f"The X-Axis Parameters are Inconsistent, Speed: {ch0_speed} and {ch1_speed},  MicroStep: {ch0_ms} and {ch1_ms}")

    sub_param = rospy.Subscriber("sub_param",slider,callback_sub_param,queue_size=50)
    sub_noparam = rospy.Subscriber("sub_noparam",operation,callback_sub_Noparam,queue_size=50)  
    rospy.spin()
