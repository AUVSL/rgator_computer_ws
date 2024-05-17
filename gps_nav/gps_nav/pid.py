#!/usr/bin/env python3

# Python header
import csv
import math
import numpy as np
import os
from numpy import linalg as la

# ROS header
import rclpy
from rclpy.node import Node
import pymap3d as pm

# Message header
from std_msgs.msg import String, Float32MultiArray
from dbw_msgs.msg import Dbw
from vectornav_msgs.msg import CommonGroup, InsGroup

# DDS header
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class PID(Node):

    def __init__(self):
        super().__init__('purepursuit')

        qos_control = QoSProfile(     
            # reliability=QoSReliabilityPolicy.BEST_EFFORT,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # init control frequency
        self.delta_t = 0.05

        # init publisher
        self.control_pub= self.create_publisher(Dbw, '/control_cmd', qos_control)

        # init subscriber
        # self.imu_common = self.create_subscription(CommonGroup, '/vectornav/raw/common', self.imu_common_callback, 10)
        self.imu_ins = self.create_subscription(InsGroup, '/vectornav/raw/ins', self.imu_ins_callback, 10)
        
        # init timer
        self.timer_ = self.create_timer(self.delta_t, self.timer_callback)
        
        # init control command
        cmd_msg = Dbw()
        cmd_msg.parkbrake    = 1
        cmd_msg.gear         = 0
        cmd_msg.throttle     = 0.0
        cmd_msg.steering     = 0.0
        cmd_msg.brake        = 0.0

        # init PID
        self.desired_speed = 0.8  # m/s
        self.v_x           = 0.0 
        self.x_ned         = 0.0
        self.y_ned         = 0.0  
        self.curr_yaw      = 90.0
        self.yaw           = 90.0
        self.i_error       = 0.0
        self.last_error    = 0.0
        self.kp            = 45
        self.ki            = 15
        self.kd            = 20
        self.wg            = 100.0
        self.de            = 0.0



    # def imu_common_callback(self, imu_msg):
    #     self.yaw         = imu_msg.yawpitchroll.x
    #     self.pitch       = imu_msg.yawpitchroll.y
    #     self.roll        = imu_msg.yawpitchroll.z

    #     self.gyro_x      = imu_msg.angularrate.x
    #     self.gyro_y      = imu_msg.angularrate.y
    #     self.gyro_z      = imu_msg.angularrate.z

    #     self.latitude    = imu_msg.position.x
    #     self.longitude   = imu_msg.position.y
    #     self.altitude    = imu_msg.position.z

        
    #     self.x_ned, self.y_ned, self.z_ned = pm.geodetic2enu(self.latitude, self.longitude, self.altitude, self.olat, self.olon, self.oalt)
        
    #     self.v_n         = imu_msg.velocity.x
    #     self.v_e         = imu_msg.velocity.y
    #     self.v_d         = imu_msg.velocity.z    

    #     self.a_x         = imu_msg.accel.x
    #     self.a_y         = imu_msg.accel.y
    #     self.a_z         = imu_msg.accel.z    

    #     self.ins_state   = imu_msg.insstatus


    def imu_ins_callback(self, ins_msg):
        self.v_x = ins_msg.velbody.x
        print("v_x", self.v_x)
        self.v_y = ins_msg.velbody.y
        self.v_z = ins_msg.velbody.z


    def pid(self):
        # p term
        error = self.desired_speed - self.v_x

        # d term
        d_error = (error - self.last_error)/self.delta_t

        # i term
        self.i_error += error*self.delta_t
        if self.i_error > self.wg:
            self.i_error = self.wg
        elif self.i_error < -self.wg:
            self.i_error = -self.wg

        # update error
        self.last_error = error
        
        # get control command
        # print("ctrl_cmd: ", self.kp * error + self.ki * self.i_error + self.kd * d_error)
        return self.kp * error + self.ki * self.i_error + self.kd * d_error





    def timer_callback(self):
        cmd_msg = Dbw()
        cmd_msg.parkbrake = 0
        cmd_msg.gear = 1
        # cmd_msg.throttle = self.pid()
        cmd_msg.throttle = 50.0
        cmd_msg.steering = 0.0
        cmd_msg.brake = 0.0

        self.control_pub.publish(cmd_msg)

        
        
def main():
    rclpy.init()
    tracking_node = PID()
    print("PID Controling")
    rclpy.spin(tracking_node)
    tracking_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
