import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32MultiArray
from dbw_msgs.msg import Dbw

import numpy as np

from ctypes import *
import ctypes


my_func = CDLL("/home/neousys/Workspace/src/dbw/dbw/C_library/neo_can.so")


class DBW(Node):
    def __init__(self):
        super().__init__('dbw')

        # init timer
        self.timer = self.create_timer(1, self.timer_callback)



    def timer_callback(self):
        print(my_func.can())

        






def main():
    rclpy.init()
    dbw_node = DBW()
    rclpy.spin(dbw_node)
    dbw_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
