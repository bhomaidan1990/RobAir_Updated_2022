/***********************************************************************************************************************
 *
 * Copyright (c) 2022, LIG / MARVIN Team / O. Aycard
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with
 * or without modification, are permitted provided that
 * the following conditions are met:
 *
 *    * Redistributions of source code must retain the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer.
 *    * Redistributions in binary form must reproduce the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer in the documentation
 *      and/or other materials provided with the
 *      distribution.
 *    * Neither the name of MARVIN nor the names of its
 *      contributors may be used to endorse or promote
 *      products derived from this software without
 *      specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***********************************************************************************************************************
 */
 
#pragma once

#ifndef ROBAIR_ROBOT_DETECT_MOTION_DISPLAY_H
#define ROBAIR_ROBOT_DETECT_MOTION_DISPLAY_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Bool.h"

namespace robair{

class DetectMotion {
public:
    /**
     * \brief Default Class Constructor.
     */
    DetectMotion(ros::NodeHandle& nh);

    /**
     * \brief Default Class Destructor.
     */
    virtual ~DetectMotion(){}

    /**
     * \brief Laser Data Processing.
     */
    void update();
    
    /**
     * \brief Laser Data Processing.
     */  
    void storeBackground();

    /**
     * \brief Laser Data Processing.
     */  
    void detectMotion();

    /**
     * \brief Laser Data Processing.
     */  
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan);

    /**
     * \brief Laser Data Processing.
     */  
    void robot_movingCallback(const std_msgs::Bool::ConstPtr &state);

    /**
     * \brief Laser Data Processing.
     */  
    void populateMarkerReference();

    /**
     * \brief Laser Data Processing.
     */  
    void populateMarkerTopic();

private:
    /**
     * \brief Node Handler.
     */
    ros::NodeHandle nh_;
    /**
     * \brief Laser Scan Subscriber.
     */
    ros::Subscriber sub_scan_;
    /**
     * \brief Robot Motion Subscriber.
     */
    ros::Subscriber sub_robot_moving_;
    /**
     * \brief Motion Detection Marker Publisher.
     */
    ros::Publisher pub_detect_motion_marker_;

    /**
     * \brief Number of Laser Scan Beams.
     */
    int nb_beams;
    /**
     * \brief Laser Min & Max Ranges.
     */
    float range_min, range_max;
    /**
     * \brief Angle Min, Max, Increment.
     */
    float angle_min, angle_max, angle_inc;
    /**
     * \brief Laser Radius & Angles.
     */
    float r[1000], theta[1000];
    /**
     * \brief Current Laser Points.
     */
    geometry_msgs::Point current_scan[1000];

    /**
     * \brief TODO.
     */
    bool init_robot;
    /**
     * \brief TODO.
     */
    bool stored_background;
    /**
     * \brief TODO.
     */
    float background[1000];
    /**
     * \brief TODO.
     */
    bool dynamic[1000];
    /**
     * \brief TODO.
     */
    bool current_robot_moving;
    /**
     * \brief TODO.
     */
    bool previous_robot_moving;
    /**
     * \brief Number of Points.
     */
    int nb_pts;
    /**
     * \brief Laser Points to Display.
     */ 
    geometry_msgs::Point display[1000];
    /**
     * \brief Color Messages.
     */
    std_msgs::ColorRGBA colors[1000];
    /**
     * \brief To Check if New Laser Data is Available.
     */
    bool init_laser;
    /**
     * \brief TODO.
     */
    bool first;
};
}

#endif