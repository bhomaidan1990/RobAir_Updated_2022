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

//Clustering Threshold in meters
#define CLUSTER_THRESHOLD 0.2

#include <tutorial_ros/utility.h>
#include <tutorial_ros/perform_clustering.h>

namespace robair
{
/***********************************************************************************************************************
 * Class definitions: PerformClustering
 */

/***********************************************************
 * Primary methods
 */
PerformClustering::PerformClustering(ros::NodeHandle& nh):
  nh_(nh) 
  {
    // init(nh_);
    // name_space_ = "/tutorial_ros";
    sub_scan_ = nh_.subscribe("/scan", 1, &scanCallback);
    // Preparing a topic to publish our results. This will be used by the visualization tool rviz
    pub_perform_clustering_marker_ = nh_.advertise<visualization_msgs::Marker>("perform_clustering_marker", 1);

    new_laser = false;

    //INFINTE LOOP TO COLLECT LASER DATA AND PROCESS THEM
    ros::Rate r(10);// this node will run at 10hz
    while (ros::ok()) {
        ros::spinOnce();//each callback is called once to collect new data: laser + robot_moving
        update();//processing of data
        r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }
}

void PerformClustering::update() {

    // we wait for new data of the laser and of the robot_moving_node to perform laser processing
    if ( new_laser ) {

        ROS_INFO("\n");
        ROS_INFO("New data of laser received");

        perfomClustering();//to perform clustering
    }
}

void PerformClustering::perfomClustering() {
//store in the table cluster, the cluster of each hit of the laser
//if the distance between the previous hit and the current one is higher than "CLUSTER_THRESHOLD"
//then we end the current cluster with the previous hit and start a new cluster with the current hit

    ROS_INFO("performing clustering");

    int nb_cluster = 0;//to count the number of cluster

    int start = 0; //to store the index of the start of current cluster
    int end;//to store the index of the end of the current cluster
    // graphical display of the start of the current cluster in green
    int nb_pts = 0;
    display[nb_pts] = current_scan[start];

    colors[nb_pts].r = 0;
    colors[nb_pts].g = 1;
    colors[nb_pts].b = 0;
    colors[nb_pts].a = 1.0;
    nb_pts++;

    for( int loop=1; loop<nb_beams; loop++ )//loop over all the hits
        //if euclidian DISTANCE between the previous hit and the current one is higher than "CLUSTER_THRESHOLD"
        if(distancePoints(current_scan[loop], current_scan[loop-1]) > CLUSTER_THRESHOLD)
        {
            //the previous hit is the end of the current cluster
            //we end the current cluster
            end = loop-1;

            //graphical display of the end of the current cluster in red
            display[nb_pts] = current_scan[end];

            colors[nb_pts].r = 1;
            colors[nb_pts].g = 0;
            colors[nb_pts].b = 0;
            colors[nb_pts].a = 1.0;
            nb_pts++;

            geometry_msgs::Point middle;
            middle = current_scan[(start + end)/2]; // compute the middle of the cluster

            //textual display
            ROS_INFO("cluster[%i] (%f, %f): hit[%i](%f, %f) -> hit[%i](%f, %f)", nb_cluster, middle.x, middle.y, 
                      start, current_scan[start].x, current_scan[start].y, end, current_scan[end].x, current_scan[end].y);

            //graphical display of the middle of the current cluster in blue
            display[nb_pts] = middle;

            colors[nb_pts].r = 0;
            colors[nb_pts].g = 0;
            colors[nb_pts].b = 1;
            colors[nb_pts].a = 1.0;
            nb_pts++;

            //the current hit is the start of the next cluster
            //we start the next cluster
            start = loop;

            //graphical display of the start of the current cluster in green
            display[nb_pts] = current_scan[start];

            colors[nb_pts].r = 0;
            colors[nb_pts].g = 1;
            colors[nb_pts].b = 0;
            colors[nb_pts].a = 1.0;
            nb_pts++;

        }

    //Dont forget to update and display the last cluster
    populateMarkerTopic(pub_perform_clustering_marker_, nb_pts, display, colors);
    ROS_INFO("clustering performed");
}

// Distance between two points
float PerformClustering::distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));

}

}