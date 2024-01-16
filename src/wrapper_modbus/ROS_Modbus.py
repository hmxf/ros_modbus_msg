from wrapper_modbus.d12_controller_mapping import controller_axis_mapping,date_map,oper_map,check_complete_map
from wrapper_modbus.d12_modbus_client import D12ModbusClient
import rospy


class Split_command():
    def _norm_xyz_move(self,value:list):
        x,y,z = value
        return [[0,x,0,x],[0,y],[0,z]]

        
    def _norm_xyz_speed(self,value:list):
        x,x_speed,y,y_speed,z,z_speed = value
        return [[0,x_speed,0,x,0,x_speed,0,x],[0,y_speed,0,y],[0,z_speed,0,z]]


    def _is_X(self, i, oper, value):
        if i == 0:
            address = controller_axis_mapping[i][oper - 1]
        else:
            address = controller_axis_mapping[i + 1][oper - 1]

        if len(value) < 6:
            values = self._norm_xyz_move(value)[i]
        else:
            values = self._norm_xyz_speed(value)[i]

        num_registers = date_map[oper_map[oper]][0] * (2 if i == 0 else 1)

        return address, num_registers, values


class OperCommand():
    def __init__(self,client):
        self.spcom    = Split_command()
        self.client   = client
        self.Axis_num = 3


    def ROS_multiAxis_StateRead(self,oper,value):
        """
        1 for read
        """
        for i in range(self.Axis_num):
            if value[i]:
                address, num_registers, values = self.spcom._is_X(i,oper,value)
                self.client.multiAxis_StateRead(address, num_registers)


    def ROS_multiAxis_Stop(self,oper,value):
        """
        0 for stop
        """
        for i in range(self.Axis_num):
            if not value[i]:
                address, num_registers, values = self.spcom._is_X(i,oper,value)
                self.client.multiAxis_Stop(address, values)


    def _perform_operation(self, i, oper, value, operation_func):
        address, num_registers, values = self.spcom._is_X(i, oper, value)

        rospy.set_param('complete', 0)
        self.complete = rospy.get_param("complete")

        tmp = 0
        if oper == 3 or 5:
            tmp = self.client._readRegisters(check_complete_map[i][0], check_complete_map[i][1])

        while self.complete == 0:
            operation_func(address, values, i,tmp)
            self.complete = rospy.get_param("complete")


    def ROS_multiAxis_AbsoluteMove(self, oper, value):
        for i in range(self.Axis_num):
            self._perform_operation(i, oper, value, self.client.multiAxis_AbsoluteMove)


    def ROS_multiAxis_AbsMoveSpeed(self, oper, value):
        for i in range(self.Axis_num):
            self._perform_operation(i, oper, value, self.client.multiAxis_AbsMoveSpeed)


    def ROS_multiAxis_Origin(self, oper, value):
        for i in range(self.Axis_num):
            self._perform_operation(i, oper, value, self.client.multiAxis_Origin)
