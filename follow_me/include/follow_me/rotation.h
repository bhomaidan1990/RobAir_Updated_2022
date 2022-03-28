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

#ifndef ROBAIR_ROBOT_ROTATION_H
#define ROBAIR_ROBOT_ROTATION_H

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

#endif