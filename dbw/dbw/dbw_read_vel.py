import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32MultiArray
from dbw_msgs.msg import Dbw, EEC1, VEP, EBC1, EEC2, ETC2, EAC1, VDC2, LD, LC, ASC1, TCO1, HRW, VP2, ETC5, Shutdown, CCVS

import numpy as np
import can
from can.message import Message

class DBW(Node):
    def __init__(self):
        super().__init__('dbw_read')

        # init subscriber
        self.dbw_sub = self.create_subscription(Dbw, '/control_cmd', self.dbw_callback, 10)

        # init publisher
        self.eec1_pub = self.create_publisher(EEC1, '/vehicle/eec1', 10)
        self.hrw_pub = self.create_publisher(HRW, '/vehicle/hrw', 10)
        self.vep_pub = self.create_publisher(VEP, '/vehicle/vep', 10)
        self.ebc_pub = self.create_publisher(EBC1, '/vehicle/ebc1', 10)
        self.eec2_pub = self.create_publisher(EEC2, '/vehicle/eec2', 10)
        self.etc2_pub = self.create_publisher(ETC2, '/vehicle/etc2', 10)
        self.vp2_pub = self.create_publisher(VP2, '/vehicle/vp2', 10)
        self.etc5_pub = self.create_publisher(ETC5, '/vehicle/etc5', 10)
        self.shutdown_pub = self.create_publisher(Shutdown, '/vehicle/shutdown', 10)
        self.ccvs_pub = self.create_publisher(CCVS, '/vehicle/ccvs', 10)
        self.eac1_pub = self.create_publisher(EAC1, '/vehicle/eac1', 10)
        self.vdc2_pub = self.create_publisher(VDC2, '/vehicle/vdc2', 10)
        self.ld_pub = self.create_publisher(LD, '/vehicle/ld', 10)
        self.lc_pub = self.create_publisher(LC, '/vehicle/lc', 10)
        self.asc1_pub = self.create_publisher(ASC1, '/vehicle/asc1', 10)
        self.tco1_pub = self.create_publisher(TCO1, '/vehicle/tco1', 10)

        # init timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        # init can
        self.bus = can.Bus(interface='socketcan', channel='can0', bitrate=250000)

        # init varibables
        self.count = 0

        # can messages definations
        # sent messages (arbitration_id = priority 0x18 & pgn # in hexidecimal & device address in hexicemial, data = initial hex command byte, the remaining bits are defaults with unassigned bytes being set to 0xFF or 0x00 arbitrary
        self.addressClaim = can.Message(arbitration_id=0x18EEFF2A, data=[0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80], is_extended_id=True)
        self.heartBeat    = can.Message(arbitration_id=0x18FFFF2A, data=[0x80, 0x31, 0x5D, 0x55, 0xFF, 0xFD, 0xFF, 0xFF], is_extended_id=True)
        self.propulsion   = can.Message(arbitration_id=0x18FFFF2A, data=[0x81, 0x7D, 0x00, 0x7D, 0xFF, 0xFF, 0xFF, 0xFF], is_extended_id=True)
        self.teleop       = can.Message(arbitration_id=0x18FFFF2A, data=[0x82, 0x7D, 0x00, 0x7D, 0xFF, 0xFF, 0xFF, 0xFF], is_extended_id=True)
        self.inhibitCmd   = can.Message(arbitration_id=0x0CFE5A2A, data=[0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF], is_extended_id=True)

        # read messages
        self.notifier = can.Notifier(self.bus, [self.listener_callback])

        # send address claim message
        self.bus.send(self.addressClaim)

    def timer_callback(self):
        self.bus.send(self.inhibitCmd)

        self.count += 1
        if self.count == 10:
            self.bus.send(self.heartBeat)
            print("sending heartbeat")
            self.count = 0


    def dbw_callback(self, msg):
        inhibit             = msg.parkbrake # 1 for lock; 0 for unlock
        gear                = msg.gear      # 0 for neutral; 1 for forward; 2 for backward
        throttle_percentage = msg.throttle  # 0 ~ 100 (%)
        brake_percentage    = msg.brake     # 0 ~ 100 (%)
        steering_percentage = msg.steering  # -100 ~ 100 (%), left+, right-

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

        # throttle
        self.teleop.data[1] = np.uint8(int(throttle_cmd, 16))
        
        # brake
        self.teleop.data[2] = np.uint8(int(brake_cmd, 16))
        
        # steering
        self.teleop.data[3] = np.uint8(int(steering_cmd, 16))
        

        if inhibit == 1:
            self.inhibitCmd.data[4] = 0x10
        else:
            self.inhibitCmd.data[4] = 0x00
        
        self.bus.send(self.teleop)
        self.bus.send(self.propulsion)
        
        print('sending control command')


    def listener_callback(self, msg):
        if msg.arbitration_id == 0x0CF00400:
            # EEC1, PGN 61444
            byte4 = msg.data[3]
            byte5 = msg.data[4]
            byte7 = msg.data[6]  

            engine_speed = 1.0*(byte4 + 256*byte5)/8 #rpm
            engine_stater_mode = byte7 % 16
            
            eec1_msg = EEC1()
            eec1_msg.engine_speed = engine_speed
            eec1_msg.engine_stater_mode = engine_stater_mode
            self.eec1_pub.publish(eec1_msg)

        elif msg.arbitration_id == 0x08FE6E27:
            # HRW, PGN 65134
            byte5 = msg.data[4]
            byte6 = msg.data[5]
            byte7 = msg.data[6] 
            byte8 = msg.data[7] 

            left_wheel_speed  = 1.0*(byte5 + 256*byte6)/256 # km/h
            right_wheel_speed = 1.0*(byte7 + 256*byte8)/256

            hrw_msg = HRW()
            hrw_msg.left_wheel_speed = left_wheel_speed
            hrw_msg.right_wheel_speed = right_wheel_speed
            self.hrw_pub.publish(hrw_msg)

        elif msg.arbitration_id == 0x18FEF727:
            # VEP, PGN 65271
            byte3 = msg.data[2]
            byte4 = msg.data[3]
            byte5 = msg.data[4]
            byte6 = msg.data[5]
            byte7 = msg.data[6]
            byte8 = msg.data[7]

            alt_potential = 1.0*(byte3 + 256*byte4)/20
            elec_potential = 1.0*(byte5 + 256*byte6)/20
            bat_potential = 1.0*(byte7 + 256*byte8)/20

            vep_msg = VEP()
            vep_msg.alt_potential = alt_potential
            vep_msg.elec_potential = elec_potential
            vep_msg.bat_potential = bat_potential
            self.vep_pub.publish(vep_msg)

        elif msg.arbitration_id == 0x18F00127:
            # EBC1, PGN 61444
            byte2 = msg.data[1]
            
            brake_pedal_position = 0.4*byte2/100
            
            ebc1_msg = EBC1()
            ebc1_msg.brake_pedal_position = brake_pedal_position
            self.ebc_pub.publish(ebc1_msg)

        elif msg.arbitration_id == 0x0CF00300:
            # EEC2, PGN 61443
            byte1 = msg.data[0]
            byte2 = msg.data[1]

            accel_pedal_position = 0.4*byte2/100
            accel_pedal_switch = byte1 // 64

            eec2_msg = EEC2()
            eec2_msg.accel_pedal_position = accel_pedal_position
            eec2_msg.accel_pedal_switch = accel_pedal_switch
            self.eec2_pub.publish(eec2_msg)

        elif msg.arbitration_id == 0x0CF00527:
            # ETC2, PGN 61445
            byte1 = msg.data[0]
            byte4 = msg.data[3]

            selected_gear = byte1 - 125
            current_gear = byte4 - 125

            etc2_msg = ETC2()
            etc2_msg.selected_gear = selected_gear
            etc2_msg.current_gear = current_gear
            self.etc2_pub.publish(etc2_msg)

        elif msg.arbitration_id == 0x18F00627:
            # EAC1, PGN 61446
            byte2 = msg.data[1]

            differential_lock = byte2 & 0x0C # get the 5th and 6th bits

            eac1_msg = EAC1()
            eac1_msg.differential_lock = differential_lock
            self.eac1_pub.publish(eac1_msg)

        elif msg.arbitration_id == 0x18F009F2:
            # VDC2, PGN 61449
            byte1 = msg.data[0]
            byte2 = msg.data[1]

            steering_angle = (byte1 + 256*byte2)/1024 - 31.25

            vdc2_msg = VDC2()
            vdc2_msg.steering_angle = steering_angle
            self.vdc2_pub.publish(vdc2_msg)

        elif msg.arbitration_id == 0x18FE4027:
            # LD, PGN 65088
            byte1 = msg.data[0]
            byte2 = msg.data[1]
            byte3 = msg.data[2]

            alt_beam = byte1 & 0x30 # get 3rd and 4th bits
            low_beam = byte1 & 0x0C # get 5th and 6th bits
            rotating_light = byte2 & 0x30 # get 3rd and 4th bits
            back_up_light = byte3 & 0xC0 # get 1st and 2nd bits

            ld_msg = LD()
            ld_msg.alt_beam = alt_beam
            ld_msg.low_beam = low_beam
            ld_msg.rotating_light = rotating_light
            ld_msg.back_up_light = back_up_light
            self.ld_pub.publish(ld_msg)

        elif msg.arbitration_id == 0x0CFE4127:
            # LC, PGN 65089
            byte1 = msg.data[0]
            byte2 = msg.data[1]
            byte3 = msg.data[2]

            alt_beam = byte1 & 0x30 # get 3rd and 4th bits
            low_beam = byte1 & 0x0C # get 5th and 6th bits
            rotating_light = byte2 & 0x30 # get 3rd and 4th bits
            back_up_light = byte3 & 0xC0 # get 1st and 2nd bits

            lc_msg = LC()
            lc_msg.alt_beam = alt_beam
            lc_msg.low_beam = low_beam
            lc_msg.rotating_light = rotating_light
            lc_msg.back_up_light = back_up_light
            self.ld_pub.publish(lc_msg)

        elif msg.arbitration_id == 0x0CFE5A27:
            # ASC1, PGN 65114
            byte5 = msg.data[4]

            motion_inhibit = byte5 & 0x30 # get 3rd and 4th bits

            asc1_msg = ASC1()
            asc1_msg.motion_inhibit = motion_inhibit
            self.asc1_pub.publish(asc1_msg)

        elif msg.arbitration_id == 0x0CFE6C27:
            # TCO1, PGN 65132
            byte1 = msg.data[0]
            byte4 = msg.data[3]

            drive_recognize = byte1 & 0x03
            direction_indicator = byte4 & 0x03

            tco1_msg = TCO1()
            tco1_msg.drive_recognize = drive_recognize
            tco1_msg.direction_indicator = direction_indicator
            self.tco1_pub.publish(tco1_msg)

        elif msg.arbitration_id == 0x1CFE8D27:
            # VP2, PGN 65165
            byte1 = msg.data[0]
            byte2 = msg.data[1]

            voltage = 1.0 * (byte1 + 256 * byte2) / 20

            vp2_msg = VP2()
            vp2_msg.voltage = voltage
            self.vp2_pub.publish(vp2_msg)

        elif msg.arbitration_id == 0x1CFEC327:
            # ETC5, PGN 65219
            byte2 = msg.data[1]

            reverse_dir_switch = byte2 & 0xC0 # get 1st and 2nd bits
            neutral_dir_switch = byte2 & 0x30 # get 3rd and 4th bits

            etc5_msg = ETC5()
            etc5_msg.reverse_dir_switch = reverse_dir_switch
            etc5_msg.neutral_dir_switch = neutral_dir_switch
            self.etc5_pub.publish(etc5_msg)

        elif msg.arbitration_id == 0x0CFEE400:
            # Shutdown, PGN 65252
            byte4 = msg.data[3]

            wait = byte4 & 0xC0 # get 1st and 2nd bits

            sd_msg = Shutdown()
            sd_msg.wait = wait
            self.shutdown_pub.publish(sd_msg)

        elif msg.arbitration_id == 0x18FEF127:
            # CCVS, PGN 65265
            byte1 = msg.data[0]
            byte2 = msg.data[1]
            byte3 = msg.data[2]
            byte4 = msg.data[3]

            parking_brake_switch = byte1 & 0x30 # get 3rd and 4th bits
            wheel_based_speed = 1.0*(byte2 + 256*byte3)/256
            brake_switch = byte4 & 0x0C # get 5rd and 6th bits

            ccvs_msg = CCVS()
            ccvs_msg.parking_brake_switch = parking_brake_switch
            ccvs_msg.wheel_based_speed = wheel_based_speed
            ccvs_msg.brake_switch = brake_switch
            self.ccvs_pub.publish(ccvs_msg)

        elif msg.arbitration_id == 0x18FEF727:
            #VEP, PGN 65271
            byte3 = msg.data[2]
            byte4 = msg.data[3]
            byte5 = msg.data[4]
            byte6 = msg.data[5]
            byte7 = msg.data[6]
            byte8 = msg.data[7]

            alt_potential = 1.0*(byte3 + 256*byte4)/20
            elec_potential = 1.0*(byte5 + 256*byte6)/20
            bat_potential = 1.0*(byte7 + 256*byte8)/20

            vep_msg = VEP()
            vep_msg.alt_potential = alt_potential
            vep_msg.elec_potential = elec_potential
            vep_msg.bat_potential = bat_potential
            self.vep_pub.publish(vep_msg)

        # elif msg.arbitration_id == 0x18FEE8FE:
        #     # VDS, PGN 65256
        #     byte3 = msg.data[2]
        #     byte4 = msg.data[3]
        #     # Navigation-Based Vehicle Speed 517?
        #     pass

def main():
    rclpy.init()
    dbw_node = DBW()
    rclpy.spin(dbw_node)
    dbw_node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
