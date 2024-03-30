#include <tutorial_ros/perform_clustering.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "perform_clustering_node");
    ros::NodeHandle nh("tutorial_ros");
    robair::PerformClustering basic_object(nh);

    ros::spin();

    return 0;
}