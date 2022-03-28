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

#ifndef ROBAIR_ROBOT_ACTION_H
#define ROBAIR_ROBOT_ACTION_H

#include "ros/ros.h"
#include "geometry_msgs/Point.h"

namespace robair{

class Action{
public:
    /**
     * \brief Default Class Constructor.
     */
    Action(ros::NodeHandle& nh);

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
    void initAction();

    /**
     * \brief TODO.
     */
    void computeRotation();

    /**
     * \brief TODO.
     */
    void computeTranslation();

    /**
     * \brief TODO.
     */
    void combineRotationTranslation();

    /**
     * \brief TODO.
     */
    void moveRobot();

    /**
     * \brief https://stackoverflow.com/questions/9323903/most-efficient-elegant-way-to-clip-a-number.
     */
    float clip(float n, float lower, float upper) {
        return std::max(lower, std::min(n, upper));
    }

    /**
     * \brief TODO.
     */
    void goalToReachCallback(const geometry_msgs::Point::ConstPtr &g);

    /**
     * \brief TODO.
     */
    void closestObstacleCallback(const geometry_msgs::Point::ConstPtr &obs);

private:
    /**
     * \brief Node Handler.
     */
    ros::NodeHandle nh_;

    /**
     * \brief Communication with one_moving_person_detector or person_tracker.
     */
    ros::Subscriber sub_goal_to_reach;

    /**
     * \brief Communication with obstacle_detection.
     */
    ros::Subscriber sub_obstacle_detection;

    /**
     * \brief Command Velocity.
     */
    ros::Publisher pub_cmd_vel;

    /**
     * \brief TODO.
     */
    geometry_msgs::Point goal_to_reach;

    /**
     * \brief New /goal_to_reach Flag.
     */
    bool new_goal_to_reach;

    /**
     * \brief Goal Reached Flag.
     */
    bool cond_goal;

    /**
     * \brief TODO.
     */
    //pid for rotation
    float rotation_to_do;
    
    /**
     * \brief TODO.
     */    
    float rotation_done;

    /**
     * \brief Error in Rotation.
     */
    float error_rotation;

    /**
     * \brief Rotation Reached Flag.
     */
    bool cond_rotation;

    /**
     * \brief Initial Rotation(before starting the pid for rotation control).
     */
    float initial_orientation;
    
    /**
     * \brief Current Orientation(provided by the odometer).
     */
    float current_orientation;

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

    /**
     * \brief TODO.
     */
    float translation_done;
    
    /**
     * \brief Error in Translation.
     */
    float error_translation;
    
    /**
     * \brief Translation Reached Flag.
     */
    bool cond_translation;
    
    /**
     * \brief Initial Position(before starting the pid for Position control)..
     */
    geometry_msgs::Point initial_position;
    
    /**
     * \brief Current Position(provided by the odometer)..
     */
    geometry_msgs::Point current_position;

    /**
     * \brief TODO.
     */    
    float error_integral_translation;

    /**
     * \brief TODO.
     */    
    float error_previous_translation;

    /**
     * \brief TODO.
     */    
    float translation_speed;

    /**
     * \brief TODO.
     */
    bool init_obstacle;

    /**
     * \brief TODO.
     */    
    geometry_msgs::Point closest_obstacle;  
};
}
#endif