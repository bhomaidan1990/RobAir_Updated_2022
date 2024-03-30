/**********************************************************
 * Copyright (c) 2024, GIPSA / COPERNIC Team / O. Aycard
 * All rights reserved.
 **********************************************************/

#pragma once
#include "ros/ros.h"

namespace robair
{

class LaserGraphicalDisplay {
public:
    /**
     * \brief Default Class Constructor.
     */
    LaserGraphicalDisplay(ros::NodeHandle& nh);

    /**
     * \brief Initialization.
     */
    bool init(ros::NodeHandle& nh);

    /**
     * \brief Laser Data Processing.
     */
    void update();

private:
    /**
     * \brief Node Handler.
     */
    ros::NodeHandle nh_;

    /**
     * \brief Graphical Display Marker Publisher.
     */
    ros::Publisher pub_laser_graphical_display_marker_;
};
}
