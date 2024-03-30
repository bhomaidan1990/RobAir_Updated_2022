#include <follow_me/rotation.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "rotation_node");

    ROS_INFO("(rotation_node) waiting for a /goal_to_reach");
    ros::NodeHandle nh("follow_me");
    robair::Rotation basic_object(nh);

    ros::spin();

    return 0;
}
