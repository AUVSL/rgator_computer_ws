# Rgator_supervisor_computer_ws

## ROS2 Packages:

### dbw: 
- dbw: subscribe control command
- dbw_read: subscribe control command; read CAN bus data, convert to ROS2 topic
	
### joy_control: 
  - joy_control: subscribe joystick input and publish contorl command  
    
### dbw_msgs: 
  - Dbw: define control command
  - [PGNs]: vehicle CAN bus data msg
    	
### vectornav:
  - vectornav: gnss-ins-imu driver
  - vectornav_msgs: gnss-ins-imu msg
	
### gps_nav:
  - pid: control vehicle speed only
  - pp_pid: pure pursuit for steering control, pid for speed control
  - track_log: log x,y,heading data from imu, convert to .csv file
  - path_visual: visualize track

&nbsp;

## Driver Usage:

### Joystick driver:
default joystick input frequency at 20Hz:

    ros2 run joy joy_node

set joystick input frequency as 40Hz:

    ros2 run joy joy_node --ros-args -p autorepeat_rate:=40.0

### Drive-by-wire:
control the vehicle only:

    ros2 run dbw dbw

control the vehicle and read CAN bus data:

    ros2 run dbw dbw_read

### Vectornav GNSS-INS sensor:
launch with modified configuratuion file:

    ros2 launch vectornav vectornav.launch.py



&nbsp;

## Project Application:

### Joystick control

    ros2 run joy joy_node
    ros2 run joy_control joy_control
    ros2 run dbw dbw
    
or

    ros2 launch joy_control joy_control_launch.py

### Path track

    ros2 launch vectornav vectornav.launch.py
    ros2 run dbw dbw
    ros2 run gps_nav pp_pid

### Path log

    ros2 launch vectornav vectornav.launch.py
    ros2 run gps_nav track_log

    
    
