import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32MultiArray
from dbw_msgs.msg import Dbw

import numpy as np

from ctypes import *
import ctypes


# my_func = CDLL("/home/neousys/Workspace/src/dbw/dbw/C_library/helper.so")
neo_func = CDLL("/home/neousys/Workspace/src/dbw/dbw/C_library/libwdt_dio.so")

class CAN_SETUP(ctypes.Structure):
    _fields_ = [
        ("bitRate", ctypes.c_uint32),
        ("recvConfig", ctypes.c_uint32),
        ("recvId", ctypes.c_uint32),
        ("recvMask", ctypes.c_uint32)
    ]

class CAN_MSG(ctypes.Structure):
    _fields_ = [
        ("id", ctypes.c_uint32),
        ("flags", ctypes.c_uint16),
        ("len", ctypes.c_uint8),
        ("data", ctypes.c_uint8*8)
    ]

class DBW(Node):
    def __init__(self):
        super().__init__('dbw')

        # init timer
        self.timer = self.create_timer(1, self.timer_callback)
        self.i = 0
        print("get version")

        print(neo_func.MCU_GetVersion(0x20150325))

        # self.can_setup = CAN_SETUP()
        # self.can_setup.bitRate = 250000 # 250kbps
        # self.can_setup.recvConfig = 0x00000008
        # self.can_setup.recvId = 0
        # self.can_setup.recvMask = 0

        # print(neo_func.CAN_Setup(0, self.can_setup, sizeof(self.can_setup)))
        # print("Setup finished")

        # self.tx_msg = CAN_MSG()
        # self.tx_msg.id = 0x1A0
        # self.tx_msg.len = 8

        # self.tx_msg.data[0] = 0
        # self.tx_msg.data[1] = 1
        # self.tx_msg.data[2] = 2
        # self.tx_msg.data[3] = 3
        # self.tx_msg.data[4] = 4
        # self.tx_msg.data[5] = 5
        # self.tx_msg.data[6] = 6
        # self.tx_msg.data[7] = 7


    def timer_callback(self):
        # print(my_func.square(self.i))
        self.i += 1
        # print(neo_func.CAN_Send(0, self.tx_msg, sizeof(self.tx_msg)))
        






def main():
    rclpy.init()
    dbw_node = DBW()
    rclpy.spin(dbw_node)
    neo_func.CAN_Stop(0)
    dbw_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
