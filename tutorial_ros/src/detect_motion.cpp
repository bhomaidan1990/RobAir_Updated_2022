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

//threshold for detection of motion
#define DETECTION_THRESHOLD 0.2

#include <tutorial_ros/detect_motion.h>
#include "visualization_msgs/Marker.h"
#include "ros/time.h"
#include <cmath>

namespace robair{
/***********************************************************************************************************************
 * Class definitions: DetectMotion
 */

/***********************************************************
 * Primary methods
 */
DetectMotion::DetectMotion(ros::NodeHandle& nh):
  nh_(nh) 
  {
    sub_scan_ = nh_.subscribe("/scan", 1, &DetectMotion::scanCallback, this);
    // TODO: check topic namespace!
    sub_robot_moving_ = nh_.subscribe("/robot_moving/robot_moving", 1, &DetectMotion::robotMovingCallback, this);
    // Preparing a topic to publish our results. This will be used by the visualization tool rviz
    pub_detect_motion_marker_ = nh_.advertise<visualization_msgs::Marker>("detect_motion_marker", 1); 

    new_laser = false;
    init_robot = false;
    previous_robot_moving = true;
    stored_background = false;

    // INFINTE LOOP TO COLLECT LASER DATA AND PROCESS THEM
    ros::Rate r(10);// this node will run at 10hz
    while (ros::ok()) {
        ros::spinOnce();//each callback is called once to collect new data: laser + robot_moving
        update();//processing of data
        r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }
}

void DetectMotion::update() {

    // we wait for new data of the laser and of the robot_moving_node to perform laser processing
    if ( new_laser && init_robot ) {

        ROS_INFO("\n");
        ROS_INFO("New data of laser received");
        ROS_INFO("New data of robot_moving received");

        nb_dynamic_pts = 0;
        if ( !current_robot_moving ) {
            //if the robot is not moving then we can perform moving person detection
            if (stored_background)
                detectMotion();
            // DO NOT FORGET to store the background but when ???
            if (previous_robot_moving)
                storeBackground();
            ROS_INFO("robot is not moving");
        }
        else
        {
            // IMPOSSIBLE TO DETECT MOTIONS because the base is moving
            // what is the value of dynamic table for each hit of the laser ?
            ROS_INFO("robot is moving");
        }
        previous_robot_moving = current_robot_moving;

        //graphical display of the results
        populateMarkerTopic();

    }
    else
        if ( !init_robot )
            ROS_WARN("waiting for robot_moving_node");

}
void DetectMotion::storeBackground() {
// store for each hit of the laser its range r in the background table

    ROS_INFO("storing background");
    /*== TODO: Fill here ==*/
    /*for (int loop=0; loop<nb_beams; loop++)
        background[loop] = ...;*/
    for (int loop=0; loop<nb_beams; loop++)
        background[loop] = r[loop];

    stored_background = true;

    ROS_INFO("background stored");
}

void DetectMotion::detectMotion() {

    ROS_INFO("detecting motion");

    for (int loop=0; loop<nb_beams; loop++ )
      {//loop over all the hits
      /*== TODO: Fill here ==*/
        // if the difference between ( the background and the current value r ) is higher than "detection_threshold"
        // then
        if( (background[loop]-r[loop]) > DETECTION_THRESHOLD)
            dynamic[loop] = true;//the current hit is dynamic
        else
            dynamic[loop] = false;//else its static

        if ( dynamic[loop] ) {

            ROS_INFO("hit[%i](%f, %f) is dynamic", loop, current_scan[loop].x, current_scan[loop].y);

            //display in blue of hits that are dynamic
            display[nb_dynamic_pts] = current_scan[loop];

            colors[nb_dynamic_pts].r = 1;
            colors[nb_dynamic_pts].g = 0;
            colors[nb_dynamic_pts].b = 1;
            colors[nb_dynamic_pts].a = 1.0;

            nb_dynamic_pts++;
        }
    }
    ROS_INFO("motion detected");
}

void DetectMotion::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

    new_laser = true;
    // store the important data related to laserscanner
    range_min = scan->range_min;
    range_max = scan->range_max;
    angle_min = scan->angle_min;
    angle_max = scan->angle_max;
    angle_inc = scan->angle_increment;
    nb_beams = ((-1 * angle_min) + angle_max)/angle_inc;

    // store the range and the coordinates in cartesian framework of each hit
    float beam_angle = angle_min;
    for ( int loop=0 ; loop < nb_beams; loop++, beam_angle += angle_inc ) {
        if ( ( scan->ranges[loop] < range_max ) && ( scan->ranges[loop] > range_min ) )
            r[loop] = scan->ranges[loop];
        else
            r[loop] = range_max;
        theta[loop] = beam_angle;

        //transform the scan in cartesian framework
        current_scan[loop].x = r[loop] * cos(beam_angle);
        current_scan[loop].y = r[loop] * sin(beam_angle);
        current_scan[loop].z = 0.0;
    }
}

void DetectMotion::robotMovingCallback(const std_msgs::Bool::ConstPtr& state) {
    init_robot = true;
    current_robot_moving = state->data;
}

// Draw the field of view and other references
void DetectMotion::populateMarkerReference() {

    visualization_msgs::Marker references;

    references.header.frame_id = "laser";
    references.header.stamp = ros::Time::now();
    references.ns = "example";
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

    pub_detect_motion_marker_.publish(references);
}

void DetectMotion::populateMarkerTopic(){

    visualization_msgs::Marker marker;

    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time::now();
    marker.ns = "example";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;

    marker.color.a = 1.0;

    //ROS_INFO("%i points to display", nb_dynamic_pts);
    for (int loop = 0; loop < nb_dynamic_pts; loop++) {
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

    pub_detect_motion_marker_.publish(marker);
    populateMarkerReference();
}

}
