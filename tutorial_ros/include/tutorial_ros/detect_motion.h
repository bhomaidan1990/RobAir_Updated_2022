/**********************************************************
 * Copyright (c) 2024, GIPSA / COPERNIC Team / O. Aycard
 * All rights reserved.
 **********************************************************/

#pragma once

#include "ros/ros.h"

namespace robair{

class DetectMotion {
public:
    /**
     * \brief Default Class Constructor.
     */
    DetectMotion(ros::NodeHandle& nh);

    /**
     * \brief Initialization.
     */
    bool init(ros::NodeHandle& nh);
    
    /**
     * \brief Laser Data Processing.
     */
    void update();
    
    /**
     * \brief Store Background.
     */  
    void storeBackground();

    /**
     * \brief Motion Detection.
     */  
    void motionDetection();

private:
    /**
     * \brief Node Handler.
     */
    ros::NodeHandle nh_;

    /**
     * \brief Robot Motion Subscriber.
     */
    ros::Subscriber sub_robot_moving_;
    
    /**
     * \brief Motion Detection Marker Publisher.
     */
    ros::Publisher pub_detect_motion_marker_;
    
    /**
     * \brief Flag to Check if Background is Stored.
     */
    bool stored_background;

    /**
     * \brief Background Points.
     */
    float background[1000];

    /**
     * \brief Dynamic Points.
     */
    bool dynamic[1000];
    
};
}
