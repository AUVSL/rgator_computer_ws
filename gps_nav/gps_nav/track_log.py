#!/usr/bin/env python3

# import ROS header
import rclpy
from rclpy.node import Node
import message_filters

# import msg header
from std_msgs.msg import String, Float32MultiArray
from vectornav_msgs.msg import CommonGroup, InsGroup
from dbw_msgs.msg import Dbw

# import Python header
import numpy as np
import csv
import pymap3d as pm

class TrackLoger(Node):

    def __init__(self):
        super().__init__('data_log')
        
        
        #init subscriber
        self.imu_common = self.create_subscription(CommonGroup, '/vectornav/raw/common', self.imu_common_callback, 10)

        # initial csv file
        file = open('data_log.csv', 'w', encoding='UTF8', newline='')
        self.writer = csv.writer(file)
        self.log_header = ['x', 'y', 'yaw']
        self.writer.writerow(self.log_header)

        # initial position
        # self.olat       = 40.0928563 # 0_track starting point
        # self.olon       = -88.2359994
        # self.oalt       = 202

        self.olat       = 40.0928319 # track starting point
        self.olon       = -88.2356109
        self.oalt       = 203


    def imu_common_callback(self, imu_msg):
        self.yaw         = imu_msg.yawpitchroll.x
        self.pitch       = imu_msg.yawpitchroll.y
        self.roll        = imu_msg.yawpitchroll.z

        self.gyro_x      = imu_msg.angularrate.x
        self.gyro_y      = imu_msg.angularrate.y
        self.gyro_z      = imu_msg.angularrate.z

        self.latitude    = imu_msg.position.x
        self.longitude   = imu_msg.position.y
        self.altitude    = imu_msg.position.z

        self.v_n         = imu_msg.velocity.x
        self.v_e         = imu_msg.velocity.y
        self.v_d         = imu_msg.velocity.z    

        self.a_x         = imu_msg.accel.x
        self.a_y         = imu_msg.accel.y
        self.a_z         = imu_msg.accel.z    

        # self.ins_state   = imu_msg.insstatus
        
        self.x, self.y, self.z = pm.geodetic2enu(self.latitude, self.longitude, self.altitude, self.olat, self.olon, self.oalt)


        print([self.x, self.y, self.yaw])
        self.data=[self.x, self.y, self.yaw]
        self.writer.writerow(self.data)
        


def main():
    rclpy.init()
    logger_node = TrackLoger()
    print("Start logging")
    rclpy.spin(logger_node)
    logger_node.destroy_node()
    self.df.to_csv('track.csv')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
