#! /usr/env/ python

import rospy
from pymodbus.register_read_message import ReadInputRegistersResponse
try:
    from pymodbus.client import ModbusTcpClient
except Exception as e:
    print("pymodbus does not seem to be installed.\n")
    print(e)
    exit()
from .post_threading import Post
from threading import Lock
from copy import deepcopy
from enum import Enum

class OperateDecorator (object):
    def __init__(self,registers_per_axis,ADDRESS_READ_START,ADDRESS_WRITE_START,Number_of_words,oper) -> None:
        """Static parameters 
        Args:
            registers_per_axis (uint16): 每轴x个寄存器
            ADDRESS_READ_START (uint16): 读取的寄存器的基地址
            ADDRESS_WRITE_START (uint16): 写入的寄存器的基地址
            Number_of_words (int16) :单双字
            oper (uint8): 0:只读 1:只写 2:读写
        """
        self.registers_per_axis = registers_per_axis
        self.Number_of_words =  Number_of_words  
        self.oper = oper 
        if self.oper == 0:
            self.ADDRESS_READ_START = ADDRESS_READ_START
        elif self.oper == 1:
            self.ADDRESS_WRITE_START = ADDRESS_WRITE_START
        elif self.oper == 2:
            self.ADDRESS_READ_START = ADDRESS_READ_START  
            self.ADDRESS_WRITE_START = ADDRESS_WRITE_START


class multiAxis_OperationTable(Enum):
    multiAxisStateRead           = OperateDecorator(6,12000, None,2,0)  # 多轴状态读取
    multiAxisStop                = OperateDecorator(2, None,12096,2,1)  # 多轴停止
    multiAxisRelativeMove        = OperateDecorator(2, None,12128,2,1)  # 多轴相对移动
    multiAxisAbsoluteMove        = OperateDecorator(2,12160,12160,2,2)  # 多轴绝对移动
    multiAxisRelativeSpeedMove   = OperateDecorator(4, None,12552,2,1)  # 多轴相对移动 + 速度
    multiAxisAbsoluteSpeedMove   = OperateDecorator(4, None,12616,2,1)  # 多轴绝对移动 + 速度
    multiAxisHoming              = OperateDecorator(2, None,12680,2,1)  # 多轴回原点


class BaseModbusClient():
    def __init__(self,host,port=502,rate=50,reset_registers=True):
        """
            :param host(string): Contains the IP adress of the modbus server
            :param rate(float): How often the registers on the modbusserver should be read per second
            :param reset_registers(bool): Defines if the holding registers should be reset to 0 after they have been read. Only possible if they are writeable
        """
        try:
            self.client = ModbusTcpClient(host,port)
        except Exception as e:
            rospy.logwarn("Could not get a modbus connection to the host modbus.\n %s\n", str(e))
            raise e
        self.post = Post(self)
        self.__last_output_time = rospy.get_time()

        self.__mutex = Lock()  # 定义互斥锁
        self._STOP = 0
        rospy.on_shutdown(self.closeConnection)  
    
    
    def closeConnection(self):
        """
        断开 Modbus 连接
        Closes the connection to the modbus
        """
        self.client.close()
    
    
    def _norm_xyz(self,values:list):
        x,y,z = values
        # tramsform list like [x,y,z] into [0,x,0,x,0,y,0,z]
        # DEBUG, change here if you tring on few axis
        return [0,x,0,x,0,y,0,z]
    
    
    def _norm_runspeed(self,values:list):

        x,x_speed,y,y_speed,z,z_speed = values
        return [0,x_speed,0,x,0,x_speed,0,x,0,y_speed,0,y,0,z_speed,0,z]

    
    def _writeRegisters(self,Address_start,Values):
        """
            写入 Modbus 寄存器  Writes modbus registers
            :param Address_start(int): First address of the values to write
            :param Values(int or list ): Values to write
        """
        with self.__mutex:
            try:
                if not rospy.is_shutdown() :
                    self.client.write_registers(Address_start, Values)
                    print("writing address:",Address_start,",values:",Values)
            except Exception as e:
                rospy.logwarn("Could not write values %s to address %d.\n ",str(values),address)
                rospy.logwarn("Exception %s\n",str(e))
                raise e
    
    
    def _is_None(x,y):
        if x is None:
            print("%s is None!\n",str(x))
            x = 0
    
    
    def _readRegisters(self,Address_start,num_registers):
        """
            读取 Modbus 寄存器  Reads modbus registers
            :param Address_start(int): First address of the registers to read
            :param num_registers(int):Amount of registers to read
        """
        self._is_None(Address_start)
        self._is_None(num_registers)
        tmp = None

        with self.__mutex:            
            try:
                result= self.client.read_holding_registers(Address_start,num_registers)
                tmp = result.registers
            except Exception as e:
                rospy.logwarn("Could not read on address %d. Exception: %s",Address_start,str(e))
                raise e
        return tmp 


    def _readRegisters(self,Address_start,num_registers):
        print("address:",Address_start,"num_registers",num_registers)
        result= self.client.read_holding_registers(Address_start,num_registers)
        tmp = result.registers
        print("readRegisters' tmp:",tmp)
 

    def readRegisters(self,Address_start,num_registers):
        print("address:",Address_start,"num_registers",num_registers)
        tmp = self._readRegisters(Address_start,num_registers)        
        print("readRegisters' tmp:",tmp)


    def multiAxis_StateRead(self):
        rospy.loginfo("Reading Status \n")
        num_registers = multiAxis_OperationTable.multiAxisStateRead.value.registers_per_axis*4
        address_read_start = multiAxis_OperationTable.multiAxisStateRead.value.ADDRESS_READ_START 
        result = self._readRegisters(address_read_start,num_registers)  # result:list
        print(f"X_1(CH0) 状态位:{bin(result[1])}    当前位置：{result[3]}       当前速度：{result[5]} ")
        print(f"X_2(CH1) 状态位:{bin(result[7])}    当前位置：{result[9]}       当前速度：{result[11]} ")
        print(f" y(CH2)  状态位:{bin(result[13])}   当前位置：{result[15]}      当前速度：{result[17]} ")
        print(f" z(CH3)  状态位:{bin(result[19])}   当前位置：{result[21]}      当前速度：{result[23]} ")
    

    def multiAxis_EMERGENCYSTOP(self):
        rospy.loginfo("EMERGENCY STOP\n")
        EMERGENCYSTOP_value = 15
        address_write_start = 12999
        self._writeRegisters(address_write_start,EMERGENCYSTOP_value)   
        self._STOP = 1
    
    
    def multiAxis_Stop(self,values):
        """Stop, 0 for stop 
        Args:
            values (list): x, y and z in a list, e.g. [0,1,0]
            [0,0,]
        """
        rospy.loginfo("MultiAxis Stop: Axis write 0 that needs to be stopped \n")
        values = self._norm_xyz(values)
        address_write_start = multiAxis_OperationTable.multiAxisStop.value.ADDRESS_WRITE_START
        self._writeRegisters(address_write_start,values)
        rospy.loginfo("The Specified Axis has been Stopped\n")


    def multiAxis_RelativeMove(self,values):
        rospy.loginfo("Relative Move\n")
        values = self._norm_xyz(values)
        address_write_start = multiAxis_OperationTable.multiAxisRelativeMove.value.ADDRESS_WRITE_START 
        self._writeRegisters(address_write_start,values)


    def multiAxis_AbsoluteMove(self,values):
        rospy.loginfo("Absolute Move\n")
        values = self._norm_xyz(values)
        num_registers = len(values)       
        address_write_start = multiAxis_OperationTable.multiAxisAbsoluteMove.value.ADDRESS_WRITE_START 
        address_read_start = multiAxis_OperationTable.multiAxisAbsoluteMove.value.ADDRESS_READ_START 
        self._writeRegisters(address_write_start,values)
        result = self._readRegisters(address_read_start,num_registers)
        while result != values:
            if self._STOP == 1:
                break
            rospy.sleep(1)
            result = self._readRegisters(address_read_start,num_registers)
            # self.multiAxisStateRead()
        rospy.set_param('complete',1)    


    def multiAxis_RelMoveSpeed(self,values):
        rospy.loginfo("Relative Move with Specific Running Speed\n")
        values = self._norm_runspeed(values)      
        address_write_start = multiAxis_OperationTable.multiAxisRelativeSpeedMove.value.ADDRESS_WRITE_START 
        self._writeRegisters(address_write_start,values)
    

    def multiAxis_AbsMoveSpeed(self,values):
        rospy.loginfo("Absolute Move with Specific Running Speed\n")
        values = self._norm_runspeed(values)      
        address_write_start = multiAxis_OperationTable.multiAxisAbsoluteSpeedMove.value.ADDRESS_WRITE_START 
        self._writeRegisters(address_write_start,values)


    def multiAxis_OriginAll(self):
        rospy.loginfo("Origin all Axis\n")
        Origin_value = 15
        address_write_start = 12997
        self._writeRegisters(address_write_start,Origin_value)

    
    def multiAxis_AdvanceOrigin(self,values):
        """Origin, 
        0 for the current speed returning to the origin
        value for return to the origin at this value speed
        Args:
            values (list): x, y and z in a list, e.g. [0,1000,0]
            [0,0,]
        """
        rospy.loginfo("Origin all Axis with Specific Speed\n")
        values = self._norm_xyz(values)
        address_write_start = multiAxis_OperationTable.multiAxisHoming.value.ADDRESS_WRITE_START 
        self._writeRegisters(address_write_start,values)       
