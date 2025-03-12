# UR5 Move Using MoveIt2

## Pre-Requirement
- ROS2 Humble
- Universal_Robots_ROS2_Driver 
  ```bash
  sudo apt-get install ros-humble-ur
  ```
  - Github: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble
  - Documents: https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/index.html

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
update2