import rospy
from ros_modbus_msg.msg import operation
from wrapper_modbus.d12_modbus_client import D12ModbusClient
from wrapper_modbus.d12_controller_mapping import *
from wrapper_modbus.ROS_Modbus import *


def speed_ms(ch_x:list):
    tmp_speed = client._readRegisters(ch_x[1],ch_x[2])
    tmp_ms = client._readRegisters(ch_x[3],ch_x[2])
    return tmp_speed,tmp_ms 


def callback_sub_param(msg):
    oper    = msg.operation
    A_x     = msg.A_x
    A_y     = msg.A_y
    A_z     = msg.A_z
    x_speed = msg.x_speed
    y_speed = msg.y_speed
    z_speed = msg.z_speed
    if x_speed != 0:
        rospy.loginfo("Received Subscriber Message: Operation:%d  A_x:%d x_speed:%d  A_y:%d y_speed:%d  A_z:%d z_speed:%d", oper,A_x,x_speed,A_y,y_speed,A_z,z_speed)   
    else:
        rospy.loginfo("Received Subscriber Message: Operation:%d  A_x:%d A_y:%d A_z:%d", oper,A_x,A_y,A_z)

    values = [A_x,A_y,A_z] 


    # multiAxis_EMERGENCYSTOP  所有轴的急停
    if oper == 0:
        client.multiAxis_EMERGENCYSTOP(multiAxis_EMERGENCYSTOP_data[0],multiAxis_EMERGENCYSTOP_data[1])

    # multiAxis_StateRead: eg: [0,1,0] 或 [1,0,1]
    elif oper == 1:
        opc.ROS_multiAxis_StateRead(oper,values)
    
    # multiAxis Stop   
    elif oper == 2:
        opc.ROS_multiAxis_Stop(oper,values)

    # multiAxis_AbsoluteMove  
    elif oper == 4:
        opc.ROS_multiAxis_AbsoluteMove(oper,values)

    # multiAxis_AbsRunSpeed  
    elif oper == 6:
        values = [A_x,x_speed,A_y,y_speed,A_z,z_speed] 
        opc.ROS_multiAxis_AbsMoveSpeed(oper,values)

    # multiAxis_Origin  
    elif oper == 7:
        values = [x_speed,y_speed,z_speed] 
        opc.ROS_multiAxis_AdvanceOrigin(oper,values)

    # multiAxis_Origin_all  
    elif oper == 9:
        # 该功能目前设备因电源功耗问题暂不支持！
        client.multiAxis_OriginAll(multiAxis_OriginAll_data[0],multiAxis_OriginAll_data[1])


if __name__ == '__main__':
    rospy.init_node('modbus_subscriber_client')

    # host                         = '192.168.1.222'
    # ch0                          = ['x_1',10120,2,10150]
    # ch1                          = ['x_2',10220,2,10250]
    # multiAxis_EMERGENCYSTOP_data = [12999,255]
    # multiAxis_OriginAll_data     = [12997,255]

    client = D12ModbusClient(host)
    rospy.loginfo("启动 ModBus 客户端")

    ch0_speed,ch0_ms = speed_ms(ch0)
    ch1_speed,ch1_ms = speed_ms(ch1)

    if (ch0_speed!= ch1_speed) or (ch0_ms != ch1_ms ):
        rospy.logwarn(f"The X-Axis Parameters are Inconsistent, Speed: {ch0_speed} and {ch1_speed},  MicroStep: {ch0_ms} and {ch1_ms}")
    else:
        rospy.loginfo("The X-Axis Parameters are consistent!")

    opc = OperCommand(client)

    sub_param = rospy.Subscriber("sub_param",operation,callback_sub_param,queue_size=50)
    rospy.spin()
