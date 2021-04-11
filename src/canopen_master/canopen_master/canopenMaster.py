#!/usr/bin/python3

import canopen
import struct
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from auxiliary.srv import McPower
from auxiliary.action import McMoveAbsolute, McMoveRelative, McReset, McMoveVelocity, McHalt, McSetPosition
from time import sleep
from threading import Timer, Lock
from std_msgs.msg import Float64

canopen_master_eds = '/home/zt/workspace/demo/src/canopen_master/eds/canopen_master.eds'
canopen_slave_motor_eds = '/home/zt/workspace/demo/src/canopen_master/eds/JMC_Canopen_EDS_v2.1.eds'
rosType2structType = {'uint16': 'H',
                'uint8': 'B',
                'int32': 'i'}

class axis():
    position_offset = 0
    scale = 1
    velocity_scale = 1
    actual_position = 0
    actual_velocity = 0

class canopen_master(Node):
    def __init__(self):
        super().__init__(node_name='canopenMaster')

        self._axis = axis()

        self._axis.position_offset = 0
        self._axis.scale = 360/4000
        self._axis.velocity_scale = 360/60

        self._sdo_lock = Lock()
        
        # init canopen stack
        self.CoNetwork = canopen.Network()
        self.CoNetwork.connect(bustype='canalystii', channel=0, bitrate=500000)
        self.CoMaster = canopen.LocalNode(1, canopen_master_eds)
        self.CoSlaveMotor = self.CoNetwork.add_node(2, canopen_slave_motor_eds)
        self.CoNetwork.nmt.state = 'OPERATIONAL' # all nodes change to OP state

        self._sdo_lock.acquire()
        self.CoSlaveMotor.sdo.download(0x6040, 0x00, struct.pack('H', 0x01)) # init param
        self.CoSlaveMotor.sdo.download(0x6040, 0x00, struct.pack('H', 0x03)) # init driver
        self.CoSlaveMotor.sdo.download(0x6060, 0x00, struct.pack(rosType2structType['uint8'], 0x01)) # position mode
        self.CoSlaveMotor.sdo.download(0x607A, 0x00, struct.pack(rosType2structType['int32'], 0x00)) 
        self.CoSlaveMotor.sdo.download(0x6081, 0x00, struct.pack(rosType2structType['int32'], 0x00))
        self.CoSlaveMotor.sdo.download(0x6083, 0x00, struct.pack(rosType2structType['uint16'], 0x00))
        self.CoSlaveMotor.sdo.download(0x6084, 0x00, struct.pack(rosType2structType['uint16'], 0x00))
        self.CoSlaveMotor.sdo.download(0x6040, 0x00, struct.pack('H', 0x0F)) # power up
        self._sdo_lock.release()

        # create msgs
        self.actualPosition_pub = self.create_publisher(Float64, 'ActualPosition', 1)
        self.actualVelocity_pub = self.create_publisher(Float64, 'ActualVelocity', 1)

        # create service
        self.mcPower_srv = self.create_service(McPower, 'McPower', self.McPower_cb)

        # create actions
        self.mcMoveabsolution_act = ActionServer(self, McMoveAbsolute, 'McMoveAbsolution', self.McMoveabsolution_cb)
        self.mcMoveRelative_act = ActionServer(self, McMoveRelative, 'McMoveRelative', self.McMoveRelative_cb)
        self.mcReset_act = ActionServer(self, McReset, 'McReset', self.McReset_cb)
        self.mcMoveVelocity_act = ActionServer(self, McMoveVelocity, 'McMoveVelocity', self.McMoveVelocity_cb)
        self.mcHalt_act = ActionServer(self, McHalt, 'McHalt', self.McHalt_cb)
        self.mcSetPosition_act = ActionServer(self, McSetPosition, 'McSetPosition', self.McSetPosition_cb)

        # create timer thread
        self._timer = Timer(0.5, self.motorUpdate)
        self._timer.start()

    def motorUpdate(self):
        self._sdo_lock.acquire()
        self._axis.actual_position = self.Position_Phy2Usr(int.from_bytes(self.CoSlaveMotor.sdo.upload(0x6064, 0x00), 'little', signed=True) + self._axis.position_offset)
        self._axis.actual_velocity = int.from_bytes(self.CoSlaveMotor.sdo.upload(0x606C, 0x00), 'little', signed=True) * self._axis.velocity_scale
        self._sdo_lock.release()

        msg = Float64()
        msg.data = self._axis.actual_position
        self.actualPosition_pub.publish(msg)

        velocity_msg = Float64()
        velocity_msg.data = self._axis.actual_velocity
        self.actualVelocity_pub.publish(velocity_msg)

        self._timer = Timer(0.5, self.motorUpdate)
        self._timer.start()


    def Position_Usr2Phy(self, usr_pos):
        return usr_pos / self._axis.scale
    
    def Position_Phy2Usr(self, phy_pos):
        return phy_pos * self._axis.scale

    def McPower_cb(self, request, respone):
        self.get_logger().info('Enter McPower_cb, axis %d, enable %d' % (request.axis, request.enable))
        # read controlword
        self._sdo_lock.acquire()
        control_word = self.CoSlaveMotor.sdo.upload(0x6040, 0x00)
        self._sdo_lock.release()
        control_word = int.from_bytes(control_word, 'little', signed=False)
        self.get_logger().info('Read controlWord 0x%X' % control_word)
        if request.enable == True:
            if control_word & 0x01 != 0x01:
                control_word = control_word | 0x01
                # write controlWord
                self.get_logger().info('Write controlWorld 0x%X' % control_word)
                self._sdo_lock.acquire()
                self.CoSlaveMotor.sdo.download(0x6040, 0x00, struct.pack(rosType2structType['uint16'], control_word))
                self._sdo_lock.release()
            if control_word & 0x03 != 0x03:
                control_word = control_word | 0x03
                # write controlWord
                self.get_logger().info('Write controlWorld 0x%X' % control_word)
                self._sdo_lock.acquire()
                self.CoSlaveMotor.sdo.download(0x6040, 0x00, struct.pack(rosType2structType['uint16'], control_word))
                self._sdo_lock.release()

            control_word = control_word | 0x08
        else:
            control_word = control_word & (~0x08)
        
        # write controlWord
        self.get_logger().info('Write controlWorld 0x%X' % control_word)
        self._sdo_lock.acquire()
        self.CoSlaveMotor.sdo.download(0x6040, 0x00, struct.pack(rosType2structType['uint16'], control_word))
        self._sdo_lock.release()

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

        position = self.Position_Usr2Phy(int(goal.request.position)) - self._axis.position_offset

        self._sdo_lock.acquire()
        self.CoSlaveMotor.sdo.download(0x6060, 0x00, struct.pack(rosType2structType['uint8'], 0x01))
        self.CoSlaveMotor.sdo.download(0x607A, 0x00, struct.pack(rosType2structType['int32'], int(position)))
        self.CoSlaveMotor.sdo.download(0x6081, 0x00, struct.pack(rosType2structType['int32'], int(goal.request.velocity)))
        self.CoSlaveMotor.sdo.download(0x6083, 0x00, struct.pack(rosType2structType['uint16'], int(goal.request.acceleration)))
        self.CoSlaveMotor.sdo.download(0x6084, 0x00, struct.pack(rosType2structType['uint16'], int(goal.request.deceleration)))

        self.CoSlaveMotor.sdo.download(0x6040, 0x00, struct.pack(rosType2structType['uint16'], 0x1F))
        self.CoSlaveMotor.sdo.download(0x6040, 0x00, struct.pack(rosType2structType['uint16'], 0x0F))
        self._sdo_lock.release()

        # wait done
        while True:
            self._sdo_lock.acquire()
            status = self.CoSlaveMotor.sdo.upload(0x6041, 0x00)
            self._sdo_lock.release()
            status = int.from_bytes(status, 'little', signed=False)
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

        self._sdo_lock.acquire()
        self.CoSlaveMotor.sdo.download(0x6060, 0x00, struct.pack(rosType2structType['uint8'], 0x01))
        self.CoSlaveMotor.sdo.download(0x607A, 0x00, struct.pack(rosType2structType['int32'], int(goal.request.distance)))
        self.CoSlaveMotor.sdo.download(0x6081, 0x00, struct.pack(rosType2structType['int32'], int(goal.request.velocity)))
        self.CoSlaveMotor.sdo.download(0x6083, 0x00, struct.pack(rosType2structType['uint16'], int(goal.request.acceleration)))
        self.CoSlaveMotor.sdo.download(0x6084, 0x00, struct.pack(rosType2structType['uint16'], int(goal.request.deceleration)))

        self.CoSlaveMotor.sdo.download(0x6040, 0x00, struct.pack(rosType2structType['uint16'], 0x5F))
        self.CoSlaveMotor.sdo.download(0x6040, 0x00, struct.pack(rosType2structType['uint16'], 0x4F))
        self._sdo_lock.release()

        # wait done
        while True:
            self._sdo_lock.acquire()
            status = self.CoSlaveMotor.sdo.upload(0x6041, 0x00)
            self._sdo_lock.release()
            status = int.from_bytes(status, 'little', signed=False)
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

        self._sdo_lock.acquire()
        self.CoSlaveMotor.sdo.download(0x6040, 0x00, struct.pack(rosType2structType['uint16'], 0x80))
        self._sdo_lock.release()

        # wait done
        while True:
            self._sdo_lock.acquire()
            status = self.CoSlaveMotor.sdo.upload(0x6041, 0x00)
            self._sdo_lock.release()
            status = int.from_bytes(status, 'little', signed=False)
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

    def McMoveVelocity_cb(self, goal):
        self.get_logger().info('Enter McMoveAbsolution_cb, axis %d execute %d velocity %f velocity %f acceleration %f deceleration %f' % (
            goal.request.axis, goal.request.execute, goal.request.velocity, goal.request.velocity, goal.request.acceleration, goal.request.deceleration
        ))

        feedback_msg = McMoveVelocity.Feedback()
        feedback_msg.invelocity = False
        feedback_msg.busy = True
        feedback_msg.active = True
        feedback_msg.commandaborted = False

        self._sdo_lock.acquire()
        self.CoSlaveMotor.sdo.download(0x6060, 0x00, struct.pack(rosType2structType['uint8'], 0x03)) # velocity mode
        self.CoSlaveMotor.sdo.download(0x6081, 0x00, struct.pack(rosType2structType['int32'], int(goal.request.velocity)))
        self.CoSlaveMotor.sdo.download(0x6083, 0x00, struct.pack(rosType2structType['uint16'], int(goal.request.acceleration)))
        self.CoSlaveMotor.sdo.download(0x6084, 0x00, struct.pack(rosType2structType['uint16'], int(goal.request.deceleration)))

        self.CoSlaveMotor.sdo.download(0x6040, 0x00, struct.pack(rosType2structType['uint16'], 0x0F))
        self._sdo_lock.release()

        # wait done
        while True:
            self._sdo_lock.acquire()
            status = self.CoSlaveMotor.sdo.upload(0x6041, 0x00)
            self._sdo_lock.release()
            status = int.from_bytes(status, 'little', signed=False)
            self.get_logger().info('status 0x%X' % status)
            if (status & 0x400) == 0x400:
                feedback_msg.invelocity = True
                feedback_msg.busy = False
                goal.publish_feedback(feedback_msg)
                break
            else:
                sleep(0.1)
            goal.publish_feedback(feedback_msg)
        
        # done
        goal.succeed()

        result = McMoveVelocity.Result()
        result.error = False
        result.errorid = 0

        return result

    def McHalt_cb(self, goal):
        self.get_logger().info('Enter McHalt_cb, axis %d execute %d deceleration %f' % (
            goal.request.axis, goal.request.execute, goal.request.deceleration
        ))

        # check mode
        self._sdo_lock.acquire()
        mode = self.CoSlaveMotor.sdo.upload(0x6061, 0x00)
        self._sdo_lock.release()
        mode = int.from_bytes(mode, 'little', signed=False)
        if mode != 3:
            return

        self.get_logger().info('velocity mode')
        feedback_msg = McHalt.Feedback()
        feedback_msg.done = False
        feedback_msg.busy = True
        feedback_msg.active = True
        feedback_msg.commandaborted = False

        self._sdo_lock.acquire()
        self.CoSlaveMotor.sdo.download(0x6040, 0x00, struct.pack(rosType2structType['uint16'], 0x010F))
        self._sdo_lock.release()

        # wait done
        while True:
            self._sdo_lock.acquire()
            status = self.CoSlaveMotor.sdo.upload(0x6041, 0x00)
            self._sdo_lock.release()
            status = int.from_bytes(status, 'little', signed=True)
            self.get_logger().info('status 0x%X' % status)
            if (status & 0x100) == 0x100:
                feedback_msg.done = True
                feedback_msg.busy = False
                goal.publish_feedback(feedback_msg)
                break
            else:
                sleep(0.1)
            goal.publish_feedback(feedback_msg)

        # done
        goal.succeed()

        result = McHalt.Result()
        result.error = False
        result.errorid = 0

        return result

    def McSetPosition_cb(self, goal):
        self.get_logger().info('Enter McSetPosition_cb, axis %d execute %d position %f mode %d' % (
            goal.request.axis, goal.request.execute, goal.request.position, goal.request.mode
        ))

        feedback_msg = McSetPosition.Feedback()
        feedback_msg.done = False
        feedback_msg.busy = True

        goal.publish_feedback(feedback_msg)

        result = McSetPosition.Result()

        if goal.request.execute != True:
            feedback_msg.done = False
            feedback_msg.busy = False
            goal.publish_feedback(feedback_msg)
            result.error = False
            result.errorid = 0
            goal.succeed()
            return result
        
        self._sdo_lock.acquire()
        actual_position = self.CoSlaveMotor.sdo.upload(0x6064, 0x00)
        self._sdo_lock.release()
        actual_position = int.from_bytes(actual_position, 'little', signed=True)

        if goal.request.mode == True: # changed relative
            self._axis.position_offset = actual_position + self.Position_Usr2Phy(goal.request.position)
        else:
            self._axis.position_offset = self.Position_Usr2Phy(goal.request.position) - actual_position
        self.get_logger().info('position_offset %d' % self._axis.position_offset)
        
        feedback_msg.done = True
        feedback_msg.busy = False
        goal.publish_feedback(feedback_msg)

        goal.succeed()

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
