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
#define ANGLE_RESOLUTION 5//in degrees
#define DISTANCE_TO_TRAVEL 1
#define ANGLE_TO_TRAVEL 20
#define UNCERTAINTY 0.05

#include <tutorial_ros/utility.h>
#include <localization/sensor_model.h>
#include "nav_msgs/SetMap.h"

namespace robair{
/***********************************************************************************************************************
 * Class definitions: Localization
 */

/***********************************************************
 * Primary methods
 */

SensorModel::SensorModel(ros::NodeHandle& nh):
  nh_(nh) 
  {
    init(nh_);

    //INFINTE LOOP TO COLLECT LASER DATA AND PROCESS THEM
    ros::Rate r(10);// this node will work at 10hz
    while (ros::ok()) {
        ros::spinOnce();//each callback is called once
        update();
        r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }
}

bool SensorModel::init(ros::NodeHandle& nh){
    sub_scan_ = nh.subscribe("/scan", 1, &scanCallback);
    sub_position = nh.subscribe("/initialpose", 1, &SensorModel::positionCallback, this);
    // Preparing a topic to publish our results. This will be used by the visualization tool rviz
    pub_sensor_model_marker = nh.advertise<visualization_msgs::Marker>("sensor_model_marker", 1);

    // get map via RPC
    nav_msgs::GetMap::Request  req;
    ROS_INFO("Requesting the map...");
    while(!ros::service::call("static_map", req, resp)) {
      ROS_WARN("Request for map failed; trying again...");
      ros::Duration d(0.5);
      d.sleep();
    }

    new_laser = false;
    init_position = false;
    new_position = false;

    width_max = resp.map.info.width;
    height_max = resp.map.info.height;
    cell_size = resp.map.info.resolution;
    min.x = resp.map.info.origin.position.x;
    min.y = resp.map.info.origin.position.y;
    max.x = min.x + width_max*cell_size;
    max.y = min.y + height_max*cell_size;

    ROS_INFO("map loaded");
    ROS_INFO("Map: (%f, %f) -> (%f, %f) with size: %f",min.x, min.y, max.x, max.y, cell_size);
    ROS_INFO("wait for initial pose");
    return true;
}

void SensorModel::update(){
    if ( new_laser && init_position && new_position )
    {

        new_position = false;
        int score_current = sensorModel(initial_position.x, initial_position.y, initial_orientation);
        ROS_INFO("initial_position(%f, %f, %f): score = %i", initial_position.x, initial_position.y, initial_orientation*180/M_PI, score_current);
        if(nb_pts>0)
            populateMarkerTopic(pub_sensor_model_marker, nb_pts, display, colors, "map");

    }
    else
        if ( !new_laser || !init_position )
            ROS_INFO("waiting for laser data or initialization of position");
}

int SensorModel::sensorModel(float x, float y, float o)
{
//compute the score of the position (x, y, o)

    nb_pts = 0;
    // we add the current hit to the hits to display
    display[nb_pts].x = x;
    display[nb_pts].y = y;
    display[nb_pts].z = 0;

    colors[nb_pts].r = 0;
    colors[nb_pts].g = 0;
    colors[nb_pts].b = 1;
    colors[nb_pts].a = 1.0;
    nb_pts++;

    // we add the current hit to the hits to display
    display[nb_pts].x = x+cos(o);
    display[nb_pts].y = y+sin(o);
    display[nb_pts].z = 0;

    colors[nb_pts].r = 1;
    colors[nb_pts].g = 1;
    colors[nb_pts].b = 0;
    colors[nb_pts].a = 1.0;
    nb_pts++;

    //loop over the hits of the laser
    int score_current = 0;
    float beam_angle = angle_min;
    for (int loop=0 ; loop < nb_beams; loop++, beam_angle += angle_inc)
    {
        //for each hit of the laser, we compute its position in the map and check if it is occupied or not

        geometry_msgs::Point hit;
        hit.x = x + r[loop]*cos(beam_angle+o);
        hit.y = y + r[loop]*sin(beam_angle+o);

        // we add the current hit to the hits to display
        display[nb_pts] = hit;

        bool cell_occupied = false;
        //loop over the positions surronding the current hit of the laser and test if one of this cell is occupied
        for(float loop_x=hit.x-UNCERTAINTY; loop_x<=hit.x+UNCERTAINTY;loop_x += UNCERTAINTY)
            for(float loop_y=hit.y-UNCERTAINTY; loop_y<=hit.y+UNCERTAINTY;loop_y += UNCERTAINTY)
                //test if the current hit of the laser corresponds to an occupied cell
                cell_occupied = cell_occupied || ( cellValue(loop_x, loop_y) == 100 );

        if ( cell_occupied )
        {
            score_current++;

            // when matching is ok: the hit of the laser is green
            colors[nb_pts].r = 0;
            colors[nb_pts].g = 1;
            colors[nb_pts].b = 0;
            colors[nb_pts].a = 1.0;
        }
        //the current hit of the laser corresponds to a free cell
        else {
            // when matching is not ok: the hit of the laser is red
            colors[nb_pts].r = 1;
            colors[nb_pts].g = 0;
            colors[nb_pts].b = 0;
            colors[nb_pts].a = 1.0;
        }
        nb_pts++;
    }
return(score_current);
}

int SensorModel::cellValue(float x, float y) {
//returns the value of the cell corresponding to the position (x, y) in the map
//returns 100 if cell(x, y) is occupied, 0 if cell(x, y) is free

    if ( ( min.x <= x ) && ( x <= max.x ) && ( min.y <= y ) && ( y <= max.y ) ) {
        float x_cell = (x-min.x)/cell_size;
        float y_cell = (y-min.y)/cell_size;
        int x_int = x_cell;
        int y_int = y_cell;
        //ROS_INFO("cell[%f = %d][%f = %d] = %d", x_cell, x_int, y_cell, y_int, map[x_int][y_int]);
        return(resp.map.data[width_max*y_int+x_int]);
    }
    else
        return(-1);

}

void SensorModel::positionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& p) {
    init_position = true;
    new_position = true;

    initial_position.x = p->pose.pose.position.x;
    initial_position.y = p->pose.pose.position.y;
    initial_orientation = tf::getYaw(p->pose.pose.orientation);
}

}