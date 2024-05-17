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



class Visual(Node):        
        
    def __init__(self):
            filename = '/home/jamie/Workspace/RGator_ROS2_ws/src/gps_nav/waypoints/SS_track.csv'

            with open(filename) as f:
                path_points = [tuple(line) for line in csv.reader(f)]
            path_points.pop(0)
            path_points_x       = np.array([float(point[0]) for point in path_points]) # east
            path_points_y       = np.array([float(point[1]) for point in path_points]) # north
            path_points_heading = np.array([float(point[2]) for point in path_points]) # heading
            # wp_size             = len(path_points_x)
            

            plt.scatter(path_points_x, path_points_y)
            plt.xlim([-10, 70])
            plt.ylim([-40, 40])
            
            plt.plot(path_points_heading)
            plt.show()


def main(args=None):
    rclpy.init(args=args)
    path_visual = Visual()
    rclpy.spin(path_visual)
    path_visual.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
