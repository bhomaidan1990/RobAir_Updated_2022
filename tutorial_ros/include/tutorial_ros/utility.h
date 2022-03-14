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

#ifndef ROBAIR_ROBOT_UTILITY_H
#define ROBAIR_ROBOT_UTILITY_H

//
#include <signal.h>
#include <string>
#include <cmath>
// ROS
#include "ros/ros.h"
#include "ros/time.h"
//
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
//
#include "std_msgs/ColorRGBA.h"
#include "visualization_msgs/Marker.h"
//
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "std_srvs/Empty.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include "nav_msgs/Odometry.h"

namespace robair
{
/**==================
 *  Shared Variables
 *===================*/
    /**------------------------
     *  General Purpose
     *-----------------------*/
    /**
     * \brief namespace.
     */
    // std::string name_space_;

    /**------------------------
     *  Laser Scan Related
     *-----------------------*/
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

    /**------------------------
     *  Graphical Display
     *-----------------------*/

    /**
     * \brief Laser Points to Display.
     */
    geometry_msgs::Point display[1000];

    /**
     * \brief Color Messages.
     */
    std_msgs::ColorRGBA colors[1000];

    /**------------------------
     *  Motion Related
     *-----------------------*/
    /**
     * \brief Robot Motion Initialization Flag.
     */

    bool init_robot;

    /**
     * \brief Robot Currently Moving Flag.
     */    
    bool current_robot_moving;

    /**
     * \brief Robot Position.
     */    
    geometry_msgs::Point position;

    /**
     * \brief Robot Orientation.
     */
    float orientation;

    /**
     * \brief Odometry Data Flag.
     */
    bool init_odom;

    /**------------------------
     *  Subscribers
     *-----------------------*/

    /**
     * \brief Laser Scan Subscriber.
     */
    ros::Subscriber sub_scan_;

    /**
     * \brief Robot Motion Subscriber.
     */
    ros::Subscriber sub_robot_moving_;

    /**
     * \brief Odometry Subscriber.
     */
    ros::Subscriber sub_odometry_;

/**==================
 *  Shared Methods
 *===================*/

/**------------------------
 *  Callbacks
 *-----------------------*/

/**
 * \brief Laser Scan Callback.
 */
void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    new_laser = true;
    // store the important data related to laserscanner
    range_min = scan->range_min;
    range_max = scan->range_max;
    angle_min = scan->angle_min;
    angle_max = scan->angle_max;
    angle_inc = scan->angle_increment;
    nb_beams = ((-1 * angle_min) + angle_max) / angle_inc;

    // store the range and the coordinates in cartesian framework of each hit
    float beam_angle = angle_min;
    for (int loop = 0; loop < nb_beams; loop++, beam_angle += angle_inc)
    {
        if ((scan->ranges[loop] < range_max) && (scan->ranges[loop] > range_min))
            r[loop] = scan->ranges[loop];
        else
            r[loop] = range_max;
        theta[loop] = beam_angle;

        // transform the scan in cartesian framework
        current_scan[loop].x = r[loop] * cos(beam_angle);
        current_scan[loop].y = r[loop] * sin(beam_angle);
        current_scan[loop].z = 0.0;
    }    
}

/**
 * \brief Robot Motion Callback.
 */  
void robotMovingCallback(const std_msgs::Bool::ConstPtr& state) {
    init_robot = true;
    current_robot_moving = state->data;
}

/**
 * \brief Odometry Callback.
 */ 
void odomCallback(const nav_msgs::Odometry::ConstPtr& o) {
    init_odom = true;
    position.x = o->pose.pose.position.x;
    position.y = o->pose.pose.position.y;
    orientation = tf::getYaw(o->pose.pose.orientation);
}
/**-----------------------------
 *  Graphical Display Markers
 *-----------------------------*/

/**
 * \brief Marker References Publishing.
 */
void populateMarkerReference(ros::Publisher &pub) {

    visualization_msgs::Marker references;

    references.header.frame_id = "laser";
    references.header.stamp = ros::Time::now();
    references.ns = "laser_graphical_display";
    references.id = 1;
    references.type = visualization_msgs::Marker::LINE_STRIP;
    references.action = visualization_msgs::Marker::ADD;
    references.pose.orientation.w = 1;

    references.scale.x = 0.02;

    references.color.r = 1.0f;
    references.color.g = 1.0f;
    references.color.b = 1.0f;
    references.color.a = 1.0;
    geometry_msgs::Point v;

    v.x =  0.02 * cos(-2.356194);
    v.y =  0.02 * sin(-2.356194);
    v.z = 0.0;
    references.points.push_back(v);

    v.x =  5.6 * cos(-2.356194);
    v.y =  5.6 * sin(-2.356194);
    v.z = 0.0;
    references.points.push_back(v);

    float beam_angle = -2.356194 + 0.006136;
    // first and last beam are already included
    for (int i=0 ; i< 723; i++, beam_angle += 0.006136){
        v.x =  5.6 * cos(beam_angle);
        v.y =  5.6 * sin(beam_angle);
        v.z = 0.0;
        references.points.push_back(v);
    }

    v.x =  5.6 * cos(2.092350);
    v.y =  5.6 * sin(2.092350);
    v.z = 0.0;
    references.points.push_back(v);

    v.x =  0.02 * cos(2.092350);
    v.y =  0.02 * sin(2.092350);
    v.z = 0.0;
    references.points.push_back(v);

    pub.publish(references);

}

/**
 * \brief Marker Topic Publishing.
 */
void populateMarkerTopic(ros::Publisher &pub, int &nb_pts,
        geometry_msgs::Point *display, std_msgs::ColorRGBA *colors){

    visualization_msgs::Marker marker;

    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time::now();
    marker.ns = "laser_graphical_display";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;

    marker.color.a = 1.0;

    //ROS_INFO("%i points to display", nb_pts);
    //for ( int time=0; time<5; time ++)
    for (int loop = 0; loop < nb_pts; loop++) {
            geometry_msgs::Point p;
            std_msgs::ColorRGBA c;

            p.x = display[loop].x;
            p.y = display[loop].y;
            p.z = display[loop].z;

            c.r = colors[loop].r;
            c.g = colors[loop].g;
            c.b = colors[loop].b;
            c.a = colors[loop].a;

            //ROS_INFO("(%f, %f, %f) with rgba (%f, %f, %f, %f)", p.x, p.y, p.z, c.r, c.g, c.b, c.a);
            marker.points.push_back(p);
            marker.colors.push_back(c);
        }

    pub.publish(marker);
    populateMarkerReference(pub);
}

/**------------------------
 *  Miscellaneous
 *-----------------------*/

/**
 * \brief Eucledian Distance Between Two Points.
 *
 * \return float Eucledian Distance in Meters.
 */
float distancePoints(geometry_msgs::Point &pa, geometry_msgs::Point &pb) {

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));
}

/**
 * \brief Middle Point of Two Points.
 *
 * \return geometry_msgs::Point Middle Point.
 */
geometry_msgs::Point legsMiddle(geometry_msgs::Point &pa, geometry_msgs::Point &pb){
    geometry_msgs::Point tmp;
    tmp.x = (pa.x + pb.x) / 2;
    tmp.y = (pa.y + pb.y) / 2;
    return tmp;
}

}

#endif