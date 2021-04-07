#!/usr/bin/python3

import canopen
import struct
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from auxiliary.srv import McPower
from auxiliary.action import McMoveAbsolute, McMoveRelative, McReset
from time import sleep

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

        # create actions
        self.mcMoveabsolution_act = ActionServer(self, McMoveAbsolute, 'McMoveAbsolution', self.McMoveabsolution_cb)
        self.mcMoveRelative_act = ActionServer(self, McMoveRelative, 'McMoveRelative', self.McMoveRelative_cb)
        self.mcReset_act = ActionServer(self, McReset, 'McReset', self.McReset_cb)

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

    def McMoveabsolution_cb(self, goal):
        self.get_logger().info('Enter McMoveAbsolution_cb, axis %d execute %d position %f velocity %f acceleration %f deceleration %f' % (
            goal.request.axis, goal.request.execute, goal.request.position, goal.request.velocity, goal.request.acceleration, goal.request.deceleration
        ))

        feedback_msg = McMoveAbsolute.Feedback()
        feedback_msg.done = False
        feedback_msg.busy = True
        feedback_msg.avtive = True
        feedback_msg.commandaborted = False

        self.CoSlaveMotor.sdo.download(0x6060, 0x00, struct.pack(rosType2structType['uint8'], 0x01))
        self.CoSlaveMotor.sdo.download(0x607A, 0x00, struct.pack(rosType2structType['int32'], int(goal.request.position)))
        self.CoSlaveMotor.sdo.download(0x6081, 0x00, struct.pack(rosType2structType['int32'], int(goal.request.velocity)))
        self.CoSlaveMotor.sdo.download(0x6083, 0x00, struct.pack(rosType2structType['uint16'], int(goal.request.acceleration)))
        self.CoSlaveMotor.sdo.download(0x6084, 0x00, struct.pack(rosType2structType['uint16'], int(goal.request.deceleration)))

        self.CoSlaveMotor.sdo.download(0x6040, 0x00, struct.pack(rosType2structType['uint16'], 0x1F))
        self.CoSlaveMotor.sdo.download(0x6040, 0x00, struct.pack(rosType2structType['uint16'], 0x0F))

        # wait done
        while True:
            status = self.CoSlaveMotor.sdo.upload(0x6041, 0x00)
            status = int.from_bytes(status, 'little')
            self.get_logger().info('status 0x%X' % status)
            if (status & 0x400) == 0x400:
                feedback_msg.done = True
                feedback_msg.busy = False
                goal.publish_feedback(feedback_msg)
                break
            else:
                sleep(0.1)
            goal.publish_feedback(feedback_msg)
        
        # done
        goal.succeed()

        result = McMoveAbsolute.Result()
        result.error = False
        result.errorid = 0

        return result

    def McMoveRelative_cb(self, goal):
        self.get_logger().info('Enter McMoveAbsolution_cb, axis %d execute %d distance %f velocity %f acceleration %f deceleration %f' % (
            goal.request.axis, goal.request.execute, goal.request.distance, goal.request.velocity, goal.request.acceleration, goal.request.deceleration
        ))

        feedback_msg = McMoveRelative.Feedback()
        feedback_msg.done = False
        feedback_msg.busy = True
        feedback_msg.avtive = True
        feedback_msg.commandaborted = False

        self.CoSlaveMotor.sdo.download(0x6060, 0x00, struct.pack(rosType2structType['uint8'], 0x01))
        self.CoSlaveMotor.sdo.download(0x607A, 0x00, struct.pack(rosType2structType['int32'], int(goal.request.distance)))
        self.CoSlaveMotor.sdo.download(0x6081, 0x00, struct.pack(rosType2structType['int32'], int(goal.request.velocity)))
        self.CoSlaveMotor.sdo.download(0x6083, 0x00, struct.pack(rosType2structType['uint16'], int(goal.request.acceleration)))
        self.CoSlaveMotor.sdo.download(0x6084, 0x00, struct.pack(rosType2structType['uint16'], int(goal.request.deceleration)))

        self.CoSlaveMotor.sdo.download(0x6040, 0x00, struct.pack(rosType2structType['uint16'], 0x5F))
        self.CoSlaveMotor.sdo.download(0x6040, 0x00, struct.pack(rosType2structType['uint16'], 0x4F))

        # wait done
        while True:
            status = self.CoSlaveMotor.sdo.upload(0x6041, 0x00)
            status = int.from_bytes(status, 'little')
            self.get_logger().info('status 0x%X' % status)
            if (status & 0x400) == 0x400:
                feedback_msg.done = True
                feedback_msg.busy = False
                goal.publish_feedback(feedback_msg)
                break
            else:
                sleep(0.1)
            goal.publish_feedback(feedback_msg)
        
        # done
        goal.succeed()

        result = McMoveRelative.Result()
        result.error = False
        result.errorid = 0

        return result

    def McReset_cb(self, goal):
        self.get_logger().info('Enter McMoveAbsolution_cb, axis %d execute %d' % (
            goal.request.axis, goal.request.execute
        ))

        feedback_msg = McReset.Feedback()
        feedback_msg.done = False
        feedback_msg.busy = True

        self.CoSlaveMotor.sdo.download(0x6040, 0x00, struct.pack(rosType2structType['uint16'], 0x80))

        # wait done
        while True:
            status = self.CoSlaveMotor.sdo.upload(0x6041, 0x00)
            status = int.from_bytes(status, 'little')
            self.get_logger().info('status 0x%X' % status)
            if (status & 0x08) != 0x08:
                feedback_msg.done = True
                feedback_msg.busy = False
                goal.publish_feedback(feedback_msg)
                break
            else:
                sleep(0.1)
            goal.publish_feedback(feedback_msg)
        
        # done
        goal.succeed()

        result = McReset.Result()
        result.error = False
        result.errorid = 0

        return result

def main(args=None):
    rclpy.init(args=args)
    canopen_master_node = canopen_master()
    rclpy.spin(canopen_master_node)
    canopen_master_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
