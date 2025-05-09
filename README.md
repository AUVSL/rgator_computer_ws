# Rgator_supervisor_computer_ws
 supervisor computer: localization, control, and vehicle interface
 
 perception computer: sensor interface, perception, and planning


## ROS2 Packages:

### dbw: 
- dbw: subscribe control command
- dbw_read: subscribe control command; read CAN bus data, convert to ROS2 topic
	
### dbw_msgs: 
  - Dbw: define control command
  - [PGNs]: vehicle CAN bus data msg

## Driver Usage:

### Drive-by-wire:
control the vehicle only:

    ros2 run dbw dbw

control the vehicle and read CAN bus data:

    ros2 run dbw dbw_read


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

#### Terminal: Get Rgator Computer Workspace
```bash
git clone https://github.com/AUVSL/rgator_computer_ws.git
```


## Running the Program
### Terminal: Run DBW Velocity Simulator
```bash
cd ~/ros2_ws/src
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run dbw dbw_vel_sim
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
