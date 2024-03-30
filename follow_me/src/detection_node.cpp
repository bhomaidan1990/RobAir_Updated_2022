#include <follow_me/detection.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "detection_node");

    ROS_INFO("waiting for activation of detection");
    ros::NodeHandle nh("follow_me");
    robair::Detection basic_object(nh);

    ros::spin();

    return 0;
}
