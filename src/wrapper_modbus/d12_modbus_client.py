import rospy
from wrapper_modbus.BaseModbusClient import BaseModbusClient
from wrapper_modbus.d12_controller_mapping import *
from threading import Lock


def _Num_Axis(result):
    """
    计算读的通道数
    """
    return int(len(result) / date_map["multiAxisStateRead"][0])


def multiAxis_StateRead_Show(result):
    """
    输出寄存器状态
    """
    StateRead_data = StateRead_OperationTable(_Num_Axis(result))
    # print(StateRead_data)

    # 输出表头
    print("{:<15} {:<20} {:<15} {:<15}".format("通道", "状态位", "当前位置","当前速度"))
    # 输出数据行
    for item in StateRead_data:
        # print("{:<15} {:<20} {:<15} {:<15}".format("轴名", "状态位", "当前位置","当前速度"))
        status = bin(result[item["status_index"]])
        position = result[item["position_index"]]
        speed = result[item["speed_index"]]
        
        print(f"{item['name']:<15} {status:<20} {position:<15} {speed:<15}")


class D12ModbusClient(BaseModbusClient):
    def __init__(self,host):
        super().__init__(host)
        self._d12modbusclient = BaseModbusClient(host)
        self._STOP = 0   

    def multiAxis_StateRead(self,address_read_start,num_registers):
        rospy.loginfo("Reading Status \n")
        result = self._d12modbusclient._readRegisters(address_read_start,num_registers)  # result:list
        multiAxis_StateRead_Show(result)
        return result 


    def multiAxis_EMERGENCYSTOP(self,address_write_start,EMERGENCYSTOP_value):
        rospy.loginfo("EMERGENCY STOP\n")
        self._writeRegisters(address_write_start,EMERGENCYSTOP_value)   
        self._STOP = 1
        rospy.set_param('complete', 1)
    
    
    def multiAxis_Stop(self,address,value):
        """Stop, 0 for stop 
        Args:
            values (list): x, y and z in a list, e.g. [0,1,0]
            [0,0,]
        """
        rospy.loginfo("MultiAxis Stop: Axis write 0 that needs to be stopped \n")
        self._writeRegisters(address,value)
        rospy.loginfo("The Specified Axis has been Stopped\n")


    
    def multiAxis_RelativeMove(self,address,value,i,tmpo):
        rospy.loginfo("Relative Move\n")
        if not self._STOP:
            self._writeRegisters(address,value)
        self.check_complete(i,value,tmpo)
    
    
    def check_complete(self,i,value,tmpo):
        result = self._readRegisters(check_complete_map[i][0], check_complete_map[i][1])
        value = value[-2:]
        for j in range(len(value)):
            value[j] = value[j]+tmpo[j]

        while result != value:
            if self._STOP == 1:
                break
            rospy.sleep(1)
            result = self.result = self.readRegisters(check_complete_map[i][0], check_complete_map[i][1])
        rospy.set_param('complete',1) 


    def multiAxis_AbsoluteMove(self,address,value,i,tmpo):
        rospy.loginfo("Absolute Move\n")
        if not self._STOP:
            self._writeRegisters(address,value)
        self.check_complete(i,value,tmpo)

    
    def multiAxis_RelMoveSpeed(self,address,value,i,tmpo):
        rospy.loginfo("Relative Move with Specific Running Speed\n")
        if not self._STOP:
            self._writeRegisters(address,value)
        self.check_complete(i,value,tmpo)
    

    def multiAxis_AbsMoveSpeed(self,address,value,i,tmpo):
        rospy.loginfo("Absolute Move with Specific Running Speed\n")
        if not self._STOP:
            self._writeRegisters(address,value)
        self.check_complete(i,value,tmpo)

    
    def multiAxis_Origin(self,address,value,i,tmpo):
        """Origin, 
        0 for the current speed returning to the origin
        value for return to the origin at this value speed
        Args:
            values (list): x, y and z in a list, e.g. [0,1000,0]
            [0,0,]
        """
        rospy.loginfo("Origin Axis with Specific Speed\n")
        if not self._STOP:
            self._writeRegisters(address,value)
        self.check_complete(i,value,tmpo)


    def multiAxis_OriginAll(self,address,value):
        """
        该功能目前设备不支持；
        """
        rospy.loginfo("Origin all Axis\n")
        self._writeRegisters(address,value)