#include <localization/sensor_model.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "compute_score_node");

    ros::NodeHandle nh("localization");
    robair::SensorModel basic_object(nh);

    ros::spin();

    return 0;
}


