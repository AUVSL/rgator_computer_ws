import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Joy
from dbw_msgs.msg import Dbw

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class Teleop(Node):

    def __init__(self):
        super().__init__('teleop')

        qos_control = QoSProfile(     
            # reliability=QoSReliabilityPolicy.BEST_EFFORT,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # init publisher
        self.control_pub= self.create_publisher(Dbw, '/control_cmd', qos_control)

        #init subscriber
        self.joystick_sub = self.create_subscription(Joy, '/joy', self.joystick_callback, 10)

        #init variables
        self.parking_brake = 1
        self.gear = 0
        self.throttle_cmd = 0.0
        self.brake_cmd = 0.0
        self.steering_cmd = 0.0


    def joystick_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.axes)

        # self.time_sec = msg.header.stamp.sec
        # self.time_nsec = msg.header.stamp.nanosec
        # print(msg.header.stamp)
        # print("the time is:", self.time_sec, "s, ", self.time_nsec, "ns")

	    # analog inputs
        # (unpressed: 1; pressed: -1)
        self.L2 = msg.axes[2] #brake
        self.R2 = msg.axes[5] #throttle

        # (left: 1; right: -1; up: 1; down: -1)
        self.L3_x = msg.axes[0] #steering
        # self.L3_y = msg.axes[1]                
        # self.R3_x = msg.axes[3]
        # self.R3_y = msg.axes[4]

	    # digital inputs 
        # (unpressed: 0; pressed: 1)
        self.cross = msg.buttons[0] # backward gear
        self.circle = msg.buttons[1] # neutral gear
        self.triangle = msg.buttons[2] # forward gear
        self.square = msg.buttons[3] # parking brake
        # self.L1 = msg.buttons[4]     
        # self.R1 = msg.buttons[5]   
        # self.L2_pressed = msg.buttons[6]
        # self.R2_pressed = msg.buttons[7]
        self.share = msg.buttons[8]     
        self.options = msg.buttons[9]
        # self.ps = msg.buttons[10]


        if self.square == 1:
            self.parking_brake = 1 # park brake engaged

        if self.share == 1 and self.options == 1:
            self.parking_brake = 0 # park brake disengaged

        
        if self.triangle == 1:
            self.gear = 1 # forward gear
        if self.circle == 1:
            self.gear = 0 # neutral gear
        if self.cross == 1:
            self.gear = 2 # backward gear  

        self.throttle_cmd = 100.0*(-self.R2 + 1)/2
        self.brake_cmd = 100.0*(-self.L2 + 1)/2
        self.steering_cmd = 100*self.L3_x

        # print([self.parking_brake, self.gear, self.throttle_cmd, self.brake_cmd, self.steering_cmd])
        cmd_msg = Dbw()
        cmd_msg.parkbrake = self.parking_brake
        cmd_msg.gear = self.gear
        cmd_msg.throttle = self.throttle_cmd
        cmd_msg.steering = self.steering_cmd
        cmd_msg.brake = self.brake_cmd

        self.control_pub.publish(cmd_msg)

        
        
def main():
    rclpy.init()
    teleop_node = Teleop()
    print("Joystick control enabled")
    rclpy.spin(teleop_node)
    teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
