#include <tutorial_ros/detect_motion.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "detect_motion_node");
    ros::NodeHandle nh("tutorial_ros");
    robair::DetectMotion basic_object(nh);

    ros::spin();

    return 0;
}
