/**********************************************************
 * Copyright (c) 2024, GIPSA / COPERNIC Team / O. Aycard
 * All rights reserved.
 **********************************************************/

#pragma once

#include "ros/ros.h"

namespace robair{

class RotationDone {
public:
    /**
     * \brief Default Class Constructor.
     */
    RotationDone(ros::NodeHandle& nh);

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
     * \brief TODO.
     */
    float rotation_done;

    /**
     * \brief Initial Orientation (ie, before starting the pid for rotation control).
     */
    float initial_orientation;

    /**
     * \brief TODO.
     */
    bool first;
};
}
