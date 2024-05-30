import rospy
from ros_modbus_msg.msg import spindle_argument
from wrapper_modbus.d12_modbus_client import D12ModbusClient


def callback_spindle_motor(msg):
    oper = msg.oper
    address = msg.address
    values = msg.value
    num_registers = msg.num_reg
    
    print(f"接受到的oper是{oper},address是{address},values是{values},num_registers是{num_registers}")
    if oper == 1:
        client._writeRegisters(address,values,16)
    else:
        client.readRegisters(address,num_registers)


if  __name__ == "__main__":
    rospy.init_node("spindle_motor")

    host = "192.168.1.222"
    client = D12ModbusClient(host)
    rospy.loginfo("启动 ModBus 客户端")

    sub_spindle_motor = rospy.Subscriber("spindle_motor",spindle_argument,callback_spindle_motor,queue_size=50)
    rospy.spin()
    
