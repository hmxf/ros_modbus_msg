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
        rospy.on_shutdown(self._closeConnection)  
    
    
    def _closeConnection(self):
        """
        断开 Modbus 连接
        Closes the connection to the modbus
        """
        self.client.close()

    
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
                    rospy.loginfo(f"BaseModbusClient-_writeRegisters: writing address:{Address_start},values:{Values}")
            except Exception as e:
                rospy.logwarn("Could not write values %s to address %d.\n ",str(Values),Address_start)
                rospy.logwarn("Exception %s\n",str(e))
                raise e
    
    
    def _is_None(self,x): 
        if x is None:
            print("%s is None!\n",str(x))
            x = 0
    
    
    def _readRegisters(self,Address_start,num_registers):
        """
            读取 Modbus 寄存器  Reads modbus registers
            :param Address_start(int): First address of the registers to read
            :param num_registers(int): Amount of registers to read
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
 

    def readRegisters(self,Address_start,num_registers):
        rospy.loginfo(f"BaseModbusClient-readRegisters: address:{Address_start}, num_registers:{num_registers}")
        tmp = self._readRegisters(Address_start,num_registers)        
        rospy.loginfo(f"BaseModbusClient-readRegisters: tmp:{tmp}")
        return tmp
