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

#ifndef ROBAIR_ROBOT_LASER_TEXT_DISPLAY_H
#define ROBAIR_ROBOT_LASER_TEXT_DISPLAY_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Bool.h"

namespace robair
{
class LaserTextDisplay {
public:
    /**
     * \brief Default Class Constructor.
     */
    LaserTextDisplay(ros::NodeHandle& nh);
    /**
     * \brief Default Class Destructor.
     */
    virtual ~LaserTextDisplay(){}
    
    /**
     * \brief Laser Data Processing.
     */    
    void update();

    /**
     * \brief Laser Scan Subscriber Callback.
     * \param scan Laser Scan Message to Trigger the Callback.
     *
     */
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan);

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
     * \brief New Laser Rading Availabilty Flag.
     */
    bool new_laser;
};
}

#endif