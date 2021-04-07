#!/usr/bin/python3

import canopen
import struct
import rclpy
from rclpy.node import Node
from auxiliary.srv import McPower

canopen_master_eds = '/home/zt/workspace/demo/src/canopen_master/eds/canopen_master.eds'
canopen_slave_motor_eds = '/home/zt/workspace/demo/src/canopen_master/eds/JMC_Canopen_EDS_v2.1.eds'
rosType2structType = {'uint16': 'H',
                'uint8': 'B',
                'int32': 'i'}

class canopen_master(Node):
    def __init__(self):
        super().__init__(node_name='canopenMaster')
        
        # init canopen stack
        self.CoNetwork = canopen.Network()
        self.CoNetwork.connect(bustype='canalystii', channel=0, bitrate=500000)
        self.CoMaster = canopen.LocalNode(1, canopen_master_eds)
        self.CoSlaveMotor = self.CoNetwork.add_node(2, canopen_slave_motor_eds)
        self.CoNetwork.nmt.state = 'OPERATIONAL' # all nodes change to OP state

        self.CoSlaveMotor.sdo.download(0x6040, 0x00, struct.pack('H', 0x01)) # init param
        self.CoSlaveMotor.sdo.download(0x6040, 0x00, struct.pack('H', 0x03)) # init driver
        self.CoSlaveMotor.sdo.download(0x6040, 0x00, struct.pack('H', 0x0F)) # power up

        # create service
        self.mcPower_srv = self.create_service(McPower, 'McPower', self.McPower_cb)

    def McPower_cb(self, request, respone):
        self.get_logger().info('Enter McPower_cb, axis %d, enable %d' % (request.axis, request.enable))
        # read controlword
        control_word = self.CoSlaveMotor.sdo.upload(0x6040, 0x00)
        control_word = int.from_bytes(control_word, 'little')
        self.get_logger().info('Read controlWord %d' % control_word)
        if request.enable == True:
            control_word = control_word | 0x08
        else:
            control_word = control_word & (~0x08)
        
        # write controlWord
        self.get_logger().info('Write controlWorld %d' % control_word)
        self.CoSlaveMotor.sdo.download(0x6040, 0x00, struct.pack(rosType2structType['uint16'], control_word))

        respone.status = False
        respone.busy = False
        respone.active = False
        respone.error = False
        respone.errorid = 0

        return respone


def main(args=None):
    rclpy.init(args=args)
    canopen_master_node = canopen_master()
    rclpy.spin(canopen_master_node)
    canopen_master_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
