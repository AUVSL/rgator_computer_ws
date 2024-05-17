#!/usr/bin/env python3

# Python header
import csv
import math
import numpy as np
import os
from numpy import linalg as la
import matplotlib.pyplot as plt

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


class PurePursuit(Node):

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
        self.imu_common = self.create_subscription(CommonGroup, '/vectornav/raw/common', self.imu_common_callback, 10)
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
        self.kp            = 45.0
        self.ki            = 15.0
        self.kd            = 20.0
        self.wg            = 100.0
        self.de            = 0.0

        # init Pure Pursuit
        self.olat       = 40.09285379 # 0_track starting point
        self.olon       = -88.23597160
        self.oalt       = 201
        self.look_ahead = 5.0 # m
        self.wheelbase  = 1.65 # meters, distance between front axle and the middle of rear axles
        self.offset     = 1.65 # meters, distance between imu and the middle of rear axles
        self.goal       = 100
        self.k          = 0.4
        # init waypoints in local NEDF
        # dirname  = os.path.dirname(__file__)
        # filename = os.path.join(dirname, '../waypoints/0_track.csv')
        filename = '/home/jamie/Workspace/RGator_ROS2_ws/src/gps_nav/waypoints/SS_track.csv'

        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]
        path_points.pop(0)
        self.path_points_x       = np.array([float(point[0]) for point in path_points]) # east
        self.path_points_y       = np.array([float(point[1]) for point in path_points]) # north
        self.path_points_heading = np.array([float(point[2]) for point in path_points]) # heading
        self.wp_size             = len(self.path_points_x)
        self.dist_arr            = np.zeros(self.wp_size)

        plt.scatter(self.path_points_x, self.path_points_y)



    def imu_common_callback(self, imu_msg):
        self.yaw         = imu_msg.yawpitchroll.x
        # print("yaw:", self.yaw)
        self.pitch       = imu_msg.yawpitchroll.y
        self.roll        = imu_msg.yawpitchroll.z

        self.gyro_x      = imu_msg.angularrate.x
        self.gyro_y      = imu_msg.angularrate.y
        self.gyro_z      = imu_msg.angularrate.z

        self.latitude    = imu_msg.position.x
        self.longitude   = imu_msg.position.y
        self.altitude    = imu_msg.position.z

        
        self.x_ned, self.y_ned, self.z_ned = pm.geodetic2enu(self.latitude, self.longitude, self.altitude, self.olat, self.olon, self.oalt)
        
        self.v_n         = imu_msg.velocity.x
        self.v_e         = imu_msg.velocity.y
        self.v_d         = imu_msg.velocity.z    

        self.a_x         = imu_msg.accel.x
        self.a_y         = imu_msg.accel.y
        self.a_z         = imu_msg.accel.z    

        self.ins_state   = imu_msg.insstatus


    def imu_ins_callback(self, ins_msg):
        self.v_x = ins_msg.velbody.x
        # print("v_x", self.v_x)
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
        return self.kp * error + self.ki * self.i_error + self.kd * d_error


    def pure_pursuit(self):
        # calculate distance array from last goal point to the end
        for i in range(self.goal, self.wp_size):
            self.dist_arr[i] = round(np.sqrt((self.path_points_x[i] - self.x_ned)**2 + (self.path_points_y[i] - self.y_ned)**2), 5)
        
        # only find point near the distance of look_ahead point
        goal_area = np.where( (self.dist_arr < self.look_ahead + 0.3) & (self.dist_arr > self.look_ahead - 0.3) )[0]

        # find goal pont in the goal area and check the direction is correct
        for idx in goal_area:
            v1 = [self.path_points_x[idx]-self.x_ned , self.path_points_y[idx]-self.y_ned]
            v2 = [np.cos(self.curr_yaw), np.sin(self.curr_yaw)]
            cosang = np.dot(v1, v2)
            sinang = la.norm(np.cross(v1, v2))
            temp_angle = np.arctan2(sinang, cosang) # [-pi, pi]

            if abs(temp_angle) < np.pi/2:
                self.goal = idx
                break

        # real look ahead distance
        L = self.dist_arr[self.goal]

        # pure pursuit control law
        alpha = self.path_points_heading[self.goal] - self.yaw 
        while alpha > 360:
            alpha -= 360
        
        while alpha < -360:
            alpha += 360        
 
        print("path heading: ", self.path_points_heading[self.goal], "yaw: ", self.yaw)
        front_wheel_angle = -2*math.atan((2*self.k*self.wheelbase*math.sin(alpha)) / L) 
        print("target: ", self.path_points_x[self.goal], self.path_points_y[self.goal], "dis:", self.dist_arr[self.goal])
        print("current: ", self.x_ned, self.y_ned)
        # print("alpha:", alpha, "front wheel angle: ", front_wheel_angle)
        # convert to control command
        steering_cmd = 100.0*(front_wheel_angle*57.3)/31.5
        if steering_cmd > 100.0:
            steering_cmd = 100.0
        if steering_cmd < -100.0:
            steering_cmd = -100.0

        print("steer_cmd", steering_cmd)

        return steering_cmd


    def timer_callback(self):
        cmd_msg = Dbw()
        cmd_msg.parkbrake = 0
        cmd_msg.gear = 1
        # cmd_msg.throttle = self.pid()
        cmd_msg.steering = self.pure_pursuit()
        cmd_msg.throttle = 50.0
        # cmd_msg.steering = 0.0
        cmd_msg.brake = 0.0

        self.control_pub.publish(cmd_msg)

        
        
def main():
    rclpy.init()
    tracking_node = PurePursuit()
    print("Pure Pursuit Controller Tracking")
    rclpy.spin(tracking_node)
    tracking_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
