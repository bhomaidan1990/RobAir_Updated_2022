/**********************************************************
 * Copyright (c) 2024, GIPSA / COPERNIC Team / O. Aycard
 * All rights reserved.
 **********************************************************/

#pragma once

#include "ros/ros.h"
#include "geometry_msgs/Point.h"

namespace robair{

class RobotMoving{
public:
    /**
     * \brief Default Class Constructor.
     */
    RobotMoving(ros::NodeHandle& nh);

    /**
     * \brief Initialization.
     */
    bool init(ros::NodeHandle& nh);
    
    /**
     * \brief Data Processing.
     */
    void update();

private:
    /**
     * \brief Node Handler.
     */
    ros::NodeHandle nh_;
    /**
     * \brief Robot Moving Publisher (to communicate with person_detector).
     */
    ros::Publisher pub_robot_moving_;

    /**
     * \brief TODO.
     */
    geometry_msgs::Point not_moving_position;

    /**
     * \brief TODO.
     */
    float not_moving_orientation;

    /**
     * \brief TODO.
     */
    int count;
    /**
     * \brief TODO.
     */
    bool moving;

};
}
