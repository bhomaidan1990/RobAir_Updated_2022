/**********************************************************
 * Copyright (c) 2024, GIPSA / COPERNIC Team / O. Aycard
 * All rights reserved.
 **********************************************************/

#pragma once

#include "ros/ros.h"
#include "geometry_msgs/Point.h"

namespace robair{

class Rotation{
public:
    /**
     * \brief Default Class Constructor.
     */
    Rotation(ros::NodeHandle& nh);

    /**
     * \brief Initialization.
     */
    bool init(ros::NodeHandle& nh);
    
    /**
     * \brief Laser Data Processing.
     */
    void update();

    /**
     * \brief TODO.
     */
    void initRotation();

    /**
     * \brief TODO.
     */
    void computeRotation();

    /**
     * \brief TODO.
     */
    void moveRobot();

    /**
     * \brief TODO.
     */
    void goalToReachCallback(const geometry_msgs::Point::ConstPtr &g);

private:
    /**
     * \brief Node Handler.
     */
    ros::NodeHandle nh_;

    /**
     * \brief TODO.
     */
    // communication with detection_node or datmo_node
    ros::Subscriber sub_goal_to_reach;

    /**
     * \brief TODO.
     */
    // communication with cmd_vel to send command to the mobile robot
    ros::Publisher pub_cmd_vel;

    /**
     * \brief TODO.
     */
    geometry_msgs::Point goal_to_reach;

    /**
     * \brief TODO.
     */
    bool new_goal_to_reach;//to check if a new /goal_to_reach is available or not

    /**
     * \brief TODO.
     */
    //pid for rotation
    float rotation_to_do, rotation_done;

    /**
     * \brief TODO.
     */
    float error_rotation;//error in rotation

    /**
     * \brief TODO.
     */
    bool cond_rotation;//boolean to check if we still have to rotate or not

    /**
     * \brief TODO.
     */
    float initial_orientation;// to store the initial orientation ie, before starting the pid for rotation control

    /**
     * \brief TODO.
     */
    float current_orientation;// to store the current orientation: this information is provided by the odometer

    /**
     * \brief TODO.
     */
    float error_integral_rotation;

    /**
     * \brief TODO.
     */
    float error_previous_rotation;

    /**
     * \brief TODO.
     */
    float rotation_speed;

    /**
     * \brief TODO.
     */
    float translation_to_do;
};
}
