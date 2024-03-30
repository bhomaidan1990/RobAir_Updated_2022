#include <localization/localization.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "localization_node");

    ros::NodeHandle nh("localization");
    robair::Localization basic_object(nh);

    ros::spin();

    return 0;
}