# UR5 Move Using MoveIt2
Move real UR5 Robot using ROS2 Humble.

## Pre-Requirement
- ROS2 Humble
- Gazebo ROS2 Package
  ```bash
  sudo apt-get install ros-humble-ros-gz
  sudo apt install ros-humble-gazebo-ros-pkgs
  ```
- Universal_Robots_ROS2_Driver 
  ```bash
  sudo apt-get install ros-humble-ur
  ```
  - Github: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble
  - Documents: https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/index.html

## System Architecture
![System Architecture](<pictures/System Architecture.png>)


**Dummy Pick Taget Environment**

![Dummy Environment Detail](<pictures/Dummy Environment Detail.png>)

Create Dummy Environment for simulate box on pallet for UR5 Robot pickup position and detection centroid of box for set pick target position to move UR5 Robot. In this project using Contour Detection algorithm to detection box and set taerget position relate to detail in picture.

**UR5 Path Planning and Execute Move UR5**

Using Universal_Robots_ROS2_Driver for connect UR5 Robot to ROS2 systerm and Using MoveIt package for create path of UR5 Robot and execute path to move real UR5 Robot using `MoveGroupInterface`.

## Network setup UR5 Robot
### Setup UR5
In Setup Robot -> Network -> Network detailed settings
```
IP address: 192.168.1.102
Subnet mask: 255.255.255.0
Default gateway: 192.168.1.1
```
### Setup PC
```
IP address: 192.168.1.101
Subnet mask: 255.255.255.0
Default gateway: 192.168.1.1
```
### Test Network Connection
```bash
ping 192.168.1.102
```
if successful output is
```
# Output
PING 192.168.1.102 (192.168.1.102) 56(84) bytes of data.
64 bytes from 192.168.1.102: icmp_seq=1 ttl=64 time=0.153 ms
64 bytes from 192.168.1.102: icmp_seq=2 ttl=64 time=0.178 ms
64 bytes from 192.168.1.102: icmp_seq=3 ttl=64 time=0.183 ms
```
### Setup UR5 EXternal Control
In UR5 program insert EXternal Control and setup in Installation -> External Control
```
Host IP: 192.168.1.101 # Using PC IP address
Custom port: 50002 # Default in Driver
Host name: EXternal Control
```
## Testing UR5 using Universal_Robots_ROS2_Driver
Run this command in terminal
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.102
```
if successful output is
```
# Output
[INFO] [spawner-7]: process has finished cleanly [pid 10600]
```
![UR5 Rviz](<pictures/Testing UR5 using Universal_Robots_ROS2_Driver.png>)
**Robot position in Rviz is related to Real UR5 Robot.**

## Using Project

### Install Project
Install project from this GitHub 

```bash
git clone https://github.com/DenWaritthon/ur_move_ws.git
```
Build and Source worckspace

```bash
cd ur_move_ws/
colcon build
source install/setup.bash
```
### How to Use this project

1. launch UR Robot Driver for connect UR5 Robot.
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 launch_rviz:=true robot_ip:=192.168.1.102
```
2. Run EXternal Control Program In Teach Pendant if successful output in terminal is.
```bash
# Output
[ur_ros2_control_node-1] [INFO] [1741780162.100902189] [UR_Client_Library:]: Robot connected to reverse interface. Ready to receive control commands.
```
3. Launch MoveIt Config for control UR5 Robot.
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=false
```
4. Run MoveIt controller node for control UR5 Robot.
```bash
ros2 run ur5_moveit ur5_moveit
```
5. Launch Dummy Station for get pick box position in gazebo.
```bash
ros2 launch camera_detection gazebo.launch.py
```
6. Run Canmera detection node for detection position of dummy box.
```bash
ros2 run camera_detection camera_detection.py
```
7. Run Target management node for management get target and set target for control UR5 Robot.
```bash
ros2 run ur5_management target_management.py
```
8. Call `/start` service for run UR5 Robot to pick-place.
```bash
ros2 service call /start target_interfaces/srv/Start "start: true"
```
9. Can move Red box in gazebo to chang pick target position and call `/start` service again
```bash
ros2 service call /start target_interfaces/srv/Start "start: true"
```
## Demo
[![Demo UR5 move Using MoveIt](<pictures/Demo UR5 move Using MoveIt.png>)](https://youtu.be/1GjvKh9H16I)
**Click to watch VDO**

## Future plan
- Deverlop showing path line before execute UR5 Robot move.
- Controll movement speed and acceleration of UR5 Robot.
- Change Dummy Environment for camera detection pick target position in gazebo to use real camera at UR5 Robot end effector.
- Add safety in work process.

## Developer Member
Waritthon Kongnoo 65340500050\
Pongpat Wongkamhaengharn 67340700402

