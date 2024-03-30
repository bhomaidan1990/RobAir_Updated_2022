#include <follow_me/obstacle_detection.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "obstacle_detection_node");
    ros::NodeHandle nh("follow_me");

    ROS_INFO("(obstacle_detection) PARAMETERS");

    float robair_size = 0.25;

    ros::param::get("/follow_me/obstacle_detection_node/robot_size", robair_size);
    ROS_INFO("(obstacle_detection) robot_size: %f", robair_size);

    robair::ObstacleDetection basic_object(nh);

    ros::spin();

    return 0;
}