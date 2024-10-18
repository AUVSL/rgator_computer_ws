import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
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
        self.lin_vel_step_size = 1/128
        self.lin_vel_min       = -250  # km/hr
        self.lin_vel_eff_max   =  40   # km/h

        self.ang_vel_step_size = 1/1024
        self.ang_vel_min       = -31.25 # rad/s
        self.ang_vel_eff_max   =  31.25 # rad/s

        self.lin_percent_range  = 100.0
        self.ang_percent_range  = 200.0
        self.ang_percent_offset = 100.0

        self.lin_offset    = abs(self.lin_vel_min / self.lin_vel_step_size)
        self.lin_spd_range = self.lin_vel_eff_max / self.lin_vel_step_size

        self.ang_offset    = abs(self.ang_vel_min / self.ang_vel_step_size)
        self.ang_spd_range = (self.ang_vel_eff_max - self.ang_vel_min) / self.ang_vel_step_size

        # can messages definations
        # sent messages (arbitration_id = priority 0x18 & pgn # in hexidecimal & device address in hexicemial, data = initial hex command byte, the remaining bits are defaults with unassigned bytes being set to 0xFF or 0x00 arbitrary
        self.addressClaim = can.Message(arbitration_id=0x18EEFF2A, data=[0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80], is_extended_id=True)
        self.heartBeat    = can.Message(arbitration_id=0x18FFFF2A, data=[0x80, 0x31, 0x5D, 0x55, 0xFF, 0xFD, 0xFF, 0xFF], is_extended_id=True)
        self.propulsion   = can.Message(arbitration_id=0x18FFFF2A, data=[0x81, 0x7D, 0x00, 0x7D, 0x00, 0xFF, 0xFF, 0xFF], is_extended_id=True)
        self.inhibitCmd   = can.Message(arbitration_id=0x0CFE5A2A, data=[0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF], is_extended_id=True)

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
        
    def set_2_bytes_number(self, byte_list, number, index):
        """Sets a 2-byte number in a list of bytes.

        Args:
            byte_list (list): The list of bytes.
            number (int): The 2-byte number to set.
            index (int): The starting index in the list to set the number.
        """

        if number < 0 or number > 65535:
            raise ValueError("Number must be between 0 and 65535")

        byte_list[index]     = number & 0xFF        # Low byte
        byte_list[index + 1] = (number >> 8) & 0xFF # High byte


    def dbw_callback(self, msg):
        inhibit             = msg.parkbrake     # 1 for lock; 0 for unlock
        gear                = msg.gear          # 0 for neutral; 1 for forward; 2 for backward
        throttle_percentage = msg.throttle      # 0 ~ 100 (%)
        steering_percentage = msg.steering      # -100 ~ 100 (%), right-, left+

        if throttle_percentage < 0:
            throttle_percentage = 0
            print("WARNING: throttle percentage below range")
        elif throttle_percentage > 100:
            throttle_percentage = 100
            print("WARNING: throttle percentage above range")

        if steering_percentage < -100:
            steering_percentage = -100
            print("WARNING: steering percentage below range")
        elif steering_percentage > 100:
            steering_percentage = 100
            print("WARNING: steering percentage above range")

        if gear == 1:
            print('gear: forward')
            throttle_cmd = np.uint16(self.lin_spd_range *  throttle_percentage / self.lin_percent_range + self.lin_offset)
        elif gear == 2:
            print('gear: backward')
            throttle_cmd = np.uint16(self.lin_spd_range * -throttle_percentage / self.lin_percent_range + self.lin_offset)
        else:
            print('gear: neutral')
            throttle_cmd = np.uint16(self.lin_offset)
        
        steering_cmd = np.uint16(self.ang_spd_range * (steering_percentage + self.ang_percent_offset) / self.ang_percent_range)

        # throttle
        self.set_2_bytes_number(self.propulsion.data, throttle_cmd, 1)
        
        # steering
        self.set_2_bytes_number(self.propulsion.data, steering_cmd, 3)

        if inhibit == 1:
            self.inhibitCmd.data[4] = 0x10
        else:
            self.inhibitCmd.data[4] = 0x00
        
        self.bus.send(self.propulsion)
        
        print('sending control command')

def main():
    rclpy.init()
    dbw_node = DBW()
    rclpy.spin(dbw_node)
    dbw_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
