# Rgator_supervisor_computer_ws
 supervisor computer: localization, control, and vehicle interface
 
 perception computer: sensor interface, perception, and planning


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

___

# ROS2 Workspace Setup and Running Instructions

This guide provides detailed instructions on how to create a ROS2 workspace, clone necessary packages, build the workspace, and run the program. Follow the steps carefully to get your code up and running.

---

## Creating the Workspace

### Step 1: Create a ROS Workspace
```bash
cd
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### Step 2: Clone Necessary Package Repositories into the `src` Folder

#### a. Path Message
```bash
git clone https://github.com/AUVSL/path_msg.git
```

#### b. Motion Controller
```bash
git clone https://github.com/AUVSL/auvsl_motion_controller
```

#### c. Vectornav Messages
```bash
git clone -b ros2 https://github.com/dawonn/vectornav.git
```

#### d. Rgator Interface Tester
```bash
git clone https://github.com/AUVSL/rgator_interface_tester.git
```

#### e. Rgator Computer Workspace
```bash
git clone https://github.com/AUVSL/rgator_computer_ws.git
```

---

### Step 3: Update the cpp_pid Package (Optional)
Inside the computer workspace file package, locate the `cpp_pid` package and update `main.cpp` as follows:

#### a. Uncomment the following constants:
```cpp
double T = 50;
double dt = 1/T;
double max = 70;
double min = -70;
double Kp = 100;
double Ki = 0;
double Kd = 0;
```

#### b. Choose one path of the three and uncomment the appropriate line:
```cpp
// Eigen::MatrixXd path {{0,0}, {10, 0}, {20, 0}, {30, 0}}; 
// Eigen::MatrixXd path {{0,0}, {1,  1}, {2,  2}, {3, 3}};
// Eigen::MatrixXd path {{0,0}, {1, -1}, {2, -2}, {3, -3}};
```

---

### Step 4: Compile the Packages
```bash
cd ~/ros2_ws/
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r
colcon build
```

---

## Running the Program

The program requires four separate terminals. Make sure to run each terminal setup in order.

### Terminal 1: Run Vectornav
```bash
cd ~/ros2_ws/src
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run vectornav vectornav
```

### Terminal 2: Run DBW Velocity Simulator
```bash
cd ~/ros2_ws/src
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run dbw dbw_vel_sim
```

### Terminal 3: Launch Motion Controller
```bash
cd ~/ros2_ws/src
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch auvsl_motion_controller outdoor.launch.py
```

### Terminal 4: Run Rgator Interface Tester
```bash
cd ~/ros2_ws/src
source /opt/ros/humble/setup.bash
source install/setup.bash
cd ~/ros2_ws/src/rgator_interface_tester/r_gator_model
python3 main.py
```

---

## Tips and Troubleshooting
- Make sure to have ROS2 Humble installed and properly configured on your system.
- If you encounter errors during the build process, try running:
  ```bash
  colcon clean
  colcon build --packages-select <package_name>
  ```
- Verify that each package has been correctly cloned and built before running.

---
