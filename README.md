# RobAir_Updated_2022
Introduction to Robotics Labs
[Course Link](https://lig-membres.imag.fr/aycard/html/Enseignement/M1/Robotics/index.html)

![RobAir](https://i.imgur.com/7ZzmMoE.png)

---

## Requirements

1. [ROS1](http://wiki.ros.org/noetic/Installation/Ubuntu).
2. [CMake](https://cmake.org/) Version >= 3.9.1.


## How it works

Considering that you have Ros1:

```
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/bhomaidan1990/RobAir_Updated_2022.git
mv RobAir_Updated_2022/* ./ && rm -rf RobAir_Updated_2022
cd .. && catkin init
catkin config -DCMAKE_BUILD_TYPE=RELEASE
catkin clean --all -y && catkin b
```
Then:

### Lab 1

```
. devel/setup.bash
roslaunch tutorial_ros robair_lab1.launch
```

### Lab 2

```
. devel/setup.bash
roslaunch follow_me robair_lab2.launch
```

### Lab 3

```
. devel/setup.bash
roslaunch localization robair_lab3.launch
```
### Gazebo Simulation
Be sure to have the `` package
`sudo apt install ros-noetic-pointcloud-to-laserscan`
Then:
```
. devel/setup.bash
roslaunch robair_gazebo robair_gazebo.launch
```

## Known Bugs:

There is delay in parsing the markers!
There is a known bug in Gazebo, so if you see some red errors don't worry, it will not be a problem.