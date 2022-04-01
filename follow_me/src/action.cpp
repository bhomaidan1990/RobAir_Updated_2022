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
#define ERROR_ROTATION_THRESHOLD 0.2//radians
#define ERROR_TRANSLATION_THRESHOLD 0.3// meters

#define ROTATION_SPEED_MAX 0.8//radians/s

#define KP_R 1
#define KI_R 0
#define KD_R 0

#define KP_T 1
#define KI_T 0
#define KD_T 0

#define SAFETY_DISTANCE 0.5

#include <tutorial_ros/utility.h>
#include <follow_me/action.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Quaternion.h"

namespace robair{
/***********************************************************************************************************************
 * Class definitions: Action
 */

/***********************************************************
 * Primary methods
 */

Action::Action(ros::NodeHandle& nh):
  nh_(nh) 
  {
    init(nh_);

    //INFINITE LOOP TO COLLECT LASER DATA AND PROCESS THEM
    ros::Rate r(10);// this node will run at 10hz
    while (ros::ok()) {
        ros::spinOnce();//each callback is called once to collect new data: laser + robot_moving
        update();//processing of data
        r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }
}

bool Action::init(ros::NodeHandle& nh){
    // communication with cmd_vel to command the mobile robot
    pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // communication with odometry
    sub_odometry_ = nh.subscribe("/odom", 1, &odomCallback);

    // communication with obstacle_detection
    sub_obstacle_detection = nh.subscribe("closest_obstacle", 1, &Action::closestObstacleCallback, this);

    // communication with datmo
    sub_goal_to_reach = nh.subscribe("goal_to_reach", 1, &Action::goalToReachCallback, this);

    cond_goal = false;
    new_goal_to_reach = false;
    init_odom = false;   
    init_obstacle = false;
    return true;
}

void Action::update() {

    if ( init_odom && init_obstacle ) { //wait for the initialization of odometer and detect_obstacle_node

        // we receive a new /goal_to_reach
        if ( new_goal_to_reach )
            initAction();

        //we are performing a rotation and a translation
        if ( cond_goal )
        {
            computeRotation();
            computeTranslation();
            cond_goal = false;
            combineRotationTranslation();
            moveRobot();
        }
    }
    else
        if ( !init_obstacle )
            ROS_WARN("waiting for obstacle_detection_node");

}

void Action::initAction()
{
    new_goal_to_reach = false;
    ROS_INFO("processing the /goal_to_reach received at (%f, %f)", goal_to_reach.x, goal_to_reach.y);

    // we have a rotation and a translation to perform
    // we compute the /translation_to_do
    translation_to_do = sqrt( ( goal_to_reach.x * goal_to_reach.x ) + ( goal_to_reach.y * goal_to_reach.y ) );

    if ( translation_to_do )
    {
        cond_goal = true;

        //we compute the /rotation_to_do
        rotation_to_do = atan2( goal_to_reach.y, goal_to_reach.x );

        // if ( goal_to_reach.y < 0 )
        //     rotation_to_do *=-1;

        //we initialize the pid for the control of rotation
        initial_orientation = current_orientation;
        error_integral_rotation = 0;
        error_previous_rotation = 0;

        //we initialize the pid for the control of translation
        initial_position = initial_position;
        error_integral_translation = 0;
        error_previous_translation = 0;

        ROS_INFO("rotation_to_do: %f, translation_to_do: %f", rotation_to_do*180/M_PI, translation_to_do);
        ROS_INFO("initial_position: (%f, %f), initial_orientation: %f provided by odometer", initial_position.x, initial_position.y, initial_orientation*180/M_PI);
    }
    else
        ROS_WARN("translation_to_do is equal to 0");

}

void Action::computeRotation()
{

    ROS_INFO("current_orientation: %f, initial_orientation: %f", current_orientation*180/M_PI, initial_orientation*180/M_PI);
    rotation_done = current_orientation - initial_orientation;

    //do not forget that rotation_done must always be between -M_PI and +M_PI

    error_rotation = current_orientation - rotation_done*M_PI/180;
    ROS_INFO("rotation_to_do: %f, rotation_done: %f, error_rotation: %f", rotation_to_do*180/M_PI, rotation_done*180/M_PI, error_rotation*180/M_PI);
    if( error_rotation < ERROR_ROTATION_THRESHOLD)
        cond_rotation = false; /*cond_rotation is used to control if we stop or not the pid*/

    rotation_speed = 0;

    if ( cond_rotation )
    {
        //Implementation of a PID controller for rotation_to_do;

        float error_derivation_rotation = error_rotation - error_previous_rotation; // divided by time-step
        error_previous_rotation = error_rotation;
        ROS_INFO("error_derivation_rotation: %f", error_derivation_rotation);

        error_integral_rotation += error_rotation;
        ROS_INFO("error_integral_rotation: %f", error_integral_rotation);

        //control of rotation with a PID controller
        rotation_speed = KP_R*error_rotation + KI_R *error_integral_rotation + KD_R * error_derivation_rotation;
        ROS_INFO("rotation_speed: %f", rotation_speed*180/M_PI);
    }
    else
        ROS_WARN("pid for rotation will stop");

}

void Action::computeTranslation()
{

    ROS_INFO("current_position: (%f, %f), initial_position: (%f, %f)", current_position.x, current_position.y, initial_position.x, initial_position.y);
    translation_done = 0;
    error_translation = 0;

    ROS_INFO("translation_to_do: %f, translation_done: %f, error_translation: %f", translation_to_do, translation_done, error_translation);

    //cond_translation = ...; cond_translation is used to control if we stop or not the pid for translation
    if(error_translation<ERROR_TRANSLATION_THRESHOLD)
        cond_translation = false;
    translation_speed = 0;

    if ( cond_translation )
    {
        //Implementation of a PID controller for translation_to_do;

        float error_derivation_translation = error_translation-error_previous_translation;
        error_previous_translation = error_translation;
        ROS_INFO("error_derivation_translation: %f", error_derivation_translation);

        error_integral_translation += error_translation;
        ROS_INFO("error_integral_translation: %f", error_integral_translation);

        //control of translation with a PID controller
        translation_speed = KP_T*error_translation + KI_T *error_integral_translation + KD_T * error_derivation_translation;
        ROS_INFO("translation_speed: %f", translation_speed);
    }
    else
        ROS_WARN("pid for translation will stop");
}

void Action::combineRotationTranslation()
{

    float coef_rotation = fabs(rotation_to_do)/ROTATION_SPEED_MAX;
    coef_rotation = clip(coef_rotation, 0, 1);
    // if (coef_rotation > 1)
    //     coef_rotation = 1;
    float coef_translation = 1 - coef_rotation;
    coef_translation = clip(coef_translation, 0, 1);

    translation_speed = translation_speed * coef_translation;

    ROS_INFO("coef_rotation: %f, rotation_speed: %f, coef_translation: %f, translation_speed: %f", coef_rotation, rotation_speed * 180 / M_PI, coef_translation, translation_speed);

    if ( translation_speed < 0 )
    {
        translation_speed = 0;
        ROS_WARN("translation_speed is negative");
    }

}

void Action::moveRobot()
{

    // processing of obstacle
    //DO NOT REMOVE
    if ( fabs(closest_obstacle.x) < SAFETY_DISTANCE )
    {
        translation_speed = 0;
        ROS_WARN("obstacle detected: (%f, %f)", closest_obstacle.x, closest_obstacle.y);
    }
    //end of processing of obstacle

    geometry_msgs::Twist twist;
    twist.linear.x = translation_speed;
    twist.linear.y = 0;
    twist.linear.z = 0;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = rotation_speed;

    pub_cmd_vel.publish(twist);

}

void Action::goalToReachCallback(const geometry_msgs::Point::ConstPtr& g) {
// process the goal received from moving_persons detector
    new_goal_to_reach = true;
    goal_to_reach = *g;
}

void Action::closestObstacleCallback(const geometry_msgs::Point::ConstPtr& obs) {
    init_obstacle = true;
    closest_obstacle = *obs;
}//closest_obstacleCallback

}