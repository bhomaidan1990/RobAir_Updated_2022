/**********************************************************
 * Copyright (c) 2024, GIPSA / COPERNIC Team / O. Aycard
 * All rights reserved.
 **********************************************************/

#pragma once

#include "ros/ros.h"
#include "geometry_msgs/Point.h"

namespace robair{

class ObstacleDetection {
public:
    /**
     * \brief Default Class Constructor.
     */
    ObstacleDetection(ros::NodeHandle& nh);

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
    // communication with action
    ros::Publisher pub_closest_obstacle;
    /**
     * \brief TODO.
     */
    ros::Publisher pub_closest_obstacle_marker;

    /**
     * \brief TODO.
     */
    geometry_msgs::Point transform_laser;

    /**
     * \brief TODO.
     */
    geometry_msgs::Point previous_closest_obstacle;
    
    /**
     * \brief TODO.
     */
    geometry_msgs::Point closest_obstacle;
};
}
