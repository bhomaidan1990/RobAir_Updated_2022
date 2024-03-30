/**********************************************************
 * Copyright (c) 2024, GIPSA / COPERNIC Team / O. Aycard
 * All rights reserved.
 **********************************************************/

#pragma once

#include "ros/ros.h"

namespace robair
{

class PerformClustering {
public:
    /**
     * \brief Default Class Constructor.
     */
    PerformClustering(ros::NodeHandle& nh);

    /**
     * \brief Initialization.
     */
    bool init(ros::NodeHandle& nh);

    /**
     * \brief Laser Data Processing.
     */
    void update();

    /**
     * \brief Laser Points Clustering.
     *
     */
    void perfomClustering();

private:
    /**
     * \brief Node Handler.
     */
    ros::NodeHandle nh_;

    /**
     * \brief Graphical Display Marker Publisher.
     */
    ros::Publisher pub_perform_clustering_marker_;

};
}
