#include <follow_me/action.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "action_node");

    ROS_INFO("(action_node) waiting for a /goal_to_reach");

    ros::NodeHandle nh("follow_me");

    robair::Action basic_object(nh);

    ros::spin();

    return 0;
}
