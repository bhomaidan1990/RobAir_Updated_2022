/**********************************************************
 * Copyright (c) 2024, GIPSA / COPERNIC Team / O. Aycard
 * All rights reserved.
 **********************************************************/

#pragma once

#include "ros/ros.h"

namespace robair
{
class LaserTextDisplay {
public:
    /**
     * \brief Default Class Constructor.
     */
    LaserTextDisplay(ros::NodeHandle& nh);

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
};
}
