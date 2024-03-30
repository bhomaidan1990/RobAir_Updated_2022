#include <tutorial_ros/robot_moving.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "robot_moving_node");
    ros::NodeHandle nh("tutorial_ros");

    ROS_INFO("(robot_moving_node) check if the robot is moving or not");
    
    robair::RobotMoving basic_object(nh);

    ros::spin();

    return 0;
}