# RobAir_Updated_2022
Introduction to Robotics Labs
[Course Link](https://lig-membres.imag.fr/aycard/html/Enseignement/M1/Robotics/index.html)

---

## Requirements

1. [ROS1](http://wiki.ros.org/noetic/Installation/Ubuntu).
2. [CMake](https://cmake.org/) Version >= 3.9.1.


## How it works

Considering that you are in valid catkin workspace:

```
catkin clean --all -y && catkin b
. devel/setup.bash
roslaunch tutorial_ros robair_lab1
```