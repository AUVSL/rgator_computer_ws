import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32MultiArray
from dbw_msgs.msg import Dbw

import numpy as np
import can

class DBW(Node):
    def __init__(self):
        super().__init__('dbw')

        # init subscriber
        self.dbw_sub = self.create_subscription(Dbw, '/control_cmd', self.dbw_callback, 10)

        # init publisher
        self.vcu_pub = self.create_publisher(Float32MultiArray, '/vcu_data', 10)

        # init timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        # init can
        self.bus = can.Bus(interface='socketcan', channel='can0', bitrate=250000)

        # init varibables
        self.count = 0

        # can messages definations
        # sent messages
        self.teleop = can.Message(arbitration_id=0x18FFFF2A, data=[0x82, 0x7D, 0x00, 0x7D, 0xFF, 0xFF, 0xFF, 0xFF], is_extended_id=True)
        self.addressClaim = can.Message(arbitration_id=0x18EEFF2A, data=[0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80], is_extended_id=True)
        self.heartBeat = can.Message(arbitration_id=0x18FFFF2A, data=[0x80, 0x31, 0x5D, 0x55, 0xFF, 0xFD, 0xFF, 0xFF], is_extended_id=True)
        self.inhibitCmd = can.Message(arbitration_id=0x0CFE5A2A, data=[0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF], is_extended_id=True)

        # read messages
        

        # send address claim message
        self.bus.send(self.addressClaim)

    def timer_callback(self):
        self.bus.send(self.inhibitCmd)

        self.count += 1
        if self.count == 10:
            self.bus.send(self.heartBeat)
            self.count = 0

        vcu_msg = Float32MultiArray()
        vcu_msg.data = []
        self.vcu_pub.publish(vcu_msg)


    def dbw_callback(self, msg):
        inhibit = msg.parkbrake                 # 1 for lock; 0 for unlock
        gear = msg.gear                         # 0 for neutral; 1 for forward; 2 for backward
        throttle_percentage = msg.throttle      # 0 ~ 100 (%)
        brake_percentage = msg.brake            # 0 ~ 100 (%)
        steering_percentage = msg.steering      # -100 ~ 100 (%), left+, right-
        # steering_percentage = 50.0

        if gear == 1:
            print('gear: forward')
            throttle_cmd = hex(int(250.0*(throttle_percentage+100)/200.))
        elif gear == 2:
            print('gear: backward')
            throttle_cmd = hex(int(250.0 * (100.0 - throttle_percentage) / 200.))
        else:
            print('gear: neutral')
            throttle_cmd = hex(int(250.0 * (0 + 100) / 200.))

        brake_cmd = hex(int(250.0*brake_percentage/100.))
        steering_cmd = hex(int(250.0*(steering_percentage+100)/200.))
    

        self.teleop.data[1] = np.uint8(int(throttle_cmd, 16))
        # throttle

        self.teleop.data[2] = np.uint8(int(brake_cmd, 16))
        # brake

        self.teleop.data[3] = np.uint8(int(steering_cmd, 16))
        # steering

        if inhibit == 1:
            self.inhibitCmd.data[4] = 0x10
        else:
            self.inhibitCmd.data[4] = 0x00
        
        self.bus.send(self.teleop)
        print('sending control command')

def main():
    rclpy.init()
    dbw_node = DBW()
    rclpy.spin(dbw_node)
    dbw_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
