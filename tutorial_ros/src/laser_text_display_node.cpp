#include <tutorial_ros/laser_text_display.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "laser_text_display_node");
    ros::NodeHandle nh("tutorial_ros");
    robair::LaserTextDisplay basic_object(nh);

    ros::spin();

    return 0;
}
