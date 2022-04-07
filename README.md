# ROS Support for Phasespace Mocap System

## Usage
1. Clone repository in your catkin workspace
```
cd ~/catkin_ws
git clone https://github.com/jayBhagiya/phasespace-mocap-ros.git
```
2. Build the packages
```
catkin_make 
# or
catkin build
```
3. Launch the mocap system
```
roslaunch phasespace_bringup phasespace_mocap.launch server_ip:="<your_mocap_system_ip>"
```