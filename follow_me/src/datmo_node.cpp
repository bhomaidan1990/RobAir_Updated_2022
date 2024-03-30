#include <follow_me/datmo.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "datmo");

    ROS_INFO("waiting for activation of datmo");
    ros::NodeHandle nh("follow_me");
    robair::DATMO basic_object(nh);

    ros::spin();

    return 0;
}
