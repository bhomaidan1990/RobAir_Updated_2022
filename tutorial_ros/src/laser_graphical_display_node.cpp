#include <tutorial_ros/laser_graphical_display.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "laser_graphical_display_node");
    ros::NodeHandle nh("tutorial_ros");
    robair::LaserGraphicalDisplay basic_object(nh);

    ros::spin();

    return 0;
}
