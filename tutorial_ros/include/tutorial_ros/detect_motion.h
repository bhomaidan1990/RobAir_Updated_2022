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

#ifndef ROBAIR_ROBOT_DETECT_MOTION_H
#define ROBAIR_ROBOT_DETECT_MOTION_H

#include "ros/ros.h"
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

    // /**
    //  * \brief Initialization.
    //  */
    // bool init(ros::NodeHandle& nh);
    
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
    void motionDetection();

    /**
     * \brief Laser Data Processing.
     */  
    void robotMovingCallback(const std_msgs::Bool::ConstPtr &state);

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
     * \brief Robot Motion Initialization Flag.
     */
    bool init_robot;

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

    /**
     * \brief Robot Motion Flag.
     */
    bool current_robot_moving;

    /**
     * \brief Robot Was in Motion Before Flag.
     */
    bool previous_robot_moving;
    
    /**
     * \brief Laser Points to Display.
     */
    geometry_msgs::Point display[1000];

    /**
     * \brief Color Messages.
     */
    std_msgs::ColorRGBA colors[1000];
};
}

#endif