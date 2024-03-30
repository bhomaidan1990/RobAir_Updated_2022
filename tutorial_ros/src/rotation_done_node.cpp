#include <tutorial_ros/rotation_done.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "rotation_done_node");
    ros::NodeHandle nh("tutorial_ros");
    robair::RotationDone basic_object(nh);

    ros::spin();

    return 0;
}


