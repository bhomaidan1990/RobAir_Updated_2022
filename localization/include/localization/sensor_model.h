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

#ifndef ROBAIR_ROBOT_SENSOR_MODEL_H
#define ROBAIR_ROBOT_SENSOR_MODEL_H

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

namespace robair
{

class SensorModel{
public:
    /**
     * \brief Default Class Constructor.
     */
    SensorModel(ros::NodeHandle& nh);

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
     * \return Something.
     */
    int sensorModel(float x, float y, float o);

    /**
     * \brief TODO.
     * \return Something.
     */
    int cellValue(float x, float y);

    /**
     * \brief TODO.
     */
    void positionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& p);

private:
    /**
     * \brief Node Handler.
     */
    ros::NodeHandle nh_;

    /**
     * \brief TODO.
     */
    ros::Subscriber sub_position;

    /**
     * \brief TODO.
     */
    ros::Publisher pub_sensor_model_marker;

    /**
     * \brief TODO.
     */
    //to store the map
    nav_msgs::GetMap::Response resp;

    /**
     * \brief TODO.
     */
    geometry_msgs::Point min, max;

    /**
     * \brief TODO.
     */
    float cell_size;

    /**
     * \brief TODO.
     */
    int width_max;

    /**
     * \brief TODO.
     */
    int height_max;

    /**
     * \brief TODO.
     */
    // GRAPHICAL DISPLAY
    int nb_pts;

    /**
     * \brief TODO.
     */
    //to store the initial_position of the mobile robot
    bool init_position;
    
    /**
     * \brief TODO.
     */
    bool new_position;

    /**
     * \brief TODO.
     */
    geometry_msgs::Point initial_position;

    /**
     * \brief TODO.
     */
    float initial_orientation;
};

}

#endif