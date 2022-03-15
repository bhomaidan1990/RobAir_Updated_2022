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
#include <tutorial_ros/utility.h>
#include <follow_me/detection.h>

namespace robair{
/***********************************************************************************************************************
 * Class definitions: Detection
 */

/***********************************************************
 * Primary methods
 */
Detection::Detection(ros::NodeHandle& nh):nh_(nh) 
{
  init(nh_);
  
  ros::Rate r(10);

  while (ros::ok()) {
      ros::spinOnce();
      update();
      r.sleep();
  }
}

bool Detection::init(ros::NodeHandle& nh){
    // To Do Some Checks and Initializations.
    ROS_INFO("Detection Node Initialization...");
    
    // name_space_ = "/follow_me";
    sub_scan_ = nh.subscribe("/scan", 1, &scanCallback);
    sub_robot_moving_ = nh.subscribe("/tutorial_ros/robot_moving", 1, &robotMovingCallback);

    // Preparing a topic to publish the goal to reach.
    pub_detection_node_ = nh.advertise<geometry_msgs::Point>("goal_to_reach", 1);

    // Preparing a topic to publish our results. This will be used by the visualization tool rviz
    pub_detection_marker_ = nh.advertise<visualization_msgs::Marker>("detection_marker", 1);

    new_laser = false;

    init_robot = false;
    
    previous_robot_moving = true;

    max_pts = 0;

    motion_detector_.setStoredBackground(false);

    return true;
}

void Detection::update()
{
  detectMotion();
  simpleClustering();
  detectLegs();
  // Visualization
  if(max_pts>0)
    populateMarkerTopic(pub_detection_marker_, max_pts, display, colors);
}

void Detection::detectMotion(){
  // we wait for new data of the laser and of the robot_moving_node to perform laser processing
  if (new_laser && init_robot)
  {
    ROS_INFO("\n");
    ROS_INFO("New data of laser received");
    ROS_INFO("New data of robot_moving received");
    // Perform Motion Detection
    motion_detector_.init(nb_beams, r, current_robot_moving, previous_robot_moving);
    motion_detector_.run();
    // Read Motion Detection Results
    for (int loop = 0; loop < nb_beams; loop++)
      dynamic[loop] = motion_detector_.getDynamicrArr()[loop];
    // Visualize the Results
    visualizeMotion();
    }
  else if (!init_robot)
    ROS_WARN("waiting for robot_moving_node");
}

void Detection::visualizeMotion(){
    // graphical display of the results
    int nb_pts = 0;
    for (int loop = 0; loop < nb_beams; loop++){
      if(dynamic[loop]){
      // Display Dynamic Hits
      display[nb_pts] = current_scan[loop];
      // Blue Color
      colors[nb_pts].r = 0;
      colors[nb_pts].g = 0;
      colors[nb_pts].b = 1;
      colors[nb_pts].a = 1.0;

      nb_pts++;
      }
    }
    if (nb_pts>max_pts)
      max_pts = nb_pts;
    // ROS_INFO("nb_pts %i", nb_pts);
    // populateMarkerTopic(pub_detection_marker_, nb_pts, display, colors);
}

void Detection::simpleClustering(){
  // Perform Clustering
  simple_clustering_.init(nb_beams, current_scan, dynamic);
  ROS_INFO("performing clustering");
  simple_clustering_.performClustering();
  // Read Clustering Results
  nb_clusters = simple_clustering_.getNumClusters();
  for (int loop = 0; loop < nb_beams; loop++)
    cluster[loop]          = simple_clustering_.getClusterArr()[loop];

  for (int c_id = 0; c_id < nb_clusters; c_id++){
    cluster_dynamic[c_id]  = simple_clustering_.getClusterDynamic()[c_id];
    cluster_middle[c_id]   = simple_clustering_.getClusterMiddle()[c_id];
    cluster_distance[c_id] = simple_clustering_.getClusterDistance()[c_id];
  }
  // Visualize the Results
  visualizeClustering();
}

void Detection::visualizeClustering(){
  //graphical display
  int nb_pts = 0;
  int start = 0;
  int end;
  int cluster_id = 0;
  // the start of the current cluster
  display[nb_pts] = current_scan[start];
  // Green Color
  colors[nb_pts].r = 0;
  colors[nb_pts].g = 1;
  colors[nb_pts].b = 0;
  colors[nb_pts].a = 1.0;
  nb_pts++;

  for( int loop=1; loop<nb_beams; loop++ )//loop over all the hits
  {
    //if (two consequtive points don't belong to the same cluster){
      //the current hit doesnt belong to the same cluster
    if(cluster[loop-1]!=cluster[loop]){
      end = loop - 1;
      // graphical display of the end of the current cluster
      display[nb_pts] = current_scan[end];
      // Red Color
      colors[nb_pts].r = 1;
      colors[nb_pts].g = 0;
      colors[nb_pts].b = 0;
      colors[nb_pts].a = 1.0;
      nb_pts++;

      //textual display
      ROS_INFO("cluster[%i] = (%f, %f): hit[%i](%f, %f) -> hit[%i](%f, %f), distance: %f, dynamic: %s", 
                                                                            cluster_id,
                                                                            cluster_middle[cluster_id].x,
                                                                            cluster_middle[cluster_id].y,
                                                                            start,
                                                                            current_scan[start].x,
                                                                            current_scan[start].y,
                                                                            end,
                                                                            current_scan[end].x,
                                                                            current_scan[end].y,
                                                                            cluster_distance[cluster_id],
                                                                            cluster_dynamic[cluster_id] ? "true" : "false");

      //graphical display of the middle of the current cluster
      display[nb_pts] = cluster_middle[cluster_id];
      // Light Blue
      colors[nb_pts].r = 0.2;
      colors[nb_pts].g = 0.2;
      colors[nb_pts].b = 1;
      colors[nb_pts].a = 1.0;
      nb_pts++;

      cluster_id++;

      // graphical display of the start of the current cluster
      start = loop;
      display[nb_pts] = current_scan[start];
      // Green
      colors[nb_pts].r = 0;
      colors[nb_pts].g = 1;
      colors[nb_pts].b = 0;
      colors[nb_pts].a = 1.0;
      nb_pts++;
    }
  }
    if (nb_pts>max_pts)
      max_pts = nb_pts;
  // ROS_INFO("nb_pts %i", nb_pts);
  // populateMarkerTopic(pub_detection_marker_, nb_pts, display, colors);
}

void Detection::detectLegs(){
  // Perform Legs Detection
  legs_detector_.init(nb_clusters, cluster_distance, cluster_middle, cluster_dynamic);
  ROS_INFO("detecting legs");
  legs_detector_.detectLegs();
  // Read Legs Detection Results
  nb_legs_detected = legs_detector_.getNumLegs();
  for (int leg_nb = 0; leg_nb < nb_legs_detected; leg_nb++)
  {
    leg_detected[leg_nb] = legs_detector_.getLegMiddle()[leg_nb]; //mid pt
    leg_cluster[leg_nb]  = legs_detector_.getLegCluster()[leg_nb]; // int arr
    leg_dynamic[leg_nb]  = legs_detector_.getLegDynamic()[leg_nb]; // bool arr
  }
  ROS_INFO("detecting legs done");
  // Visualize the Results
  visualizeLegs();
}

void Detection::visualizeLegs(){
  int nb_pts = 0;
  //loop over all the clusters
  for (int c_id = 0; c_id<nb_clusters; c_id++)
    if (cluster_middle[c_id]==leg_detected[c_id]){
      //then the current cluster is a leg
      if ( leg_dynamic[c_id] )
      {
        ROS_INFO("moving leg found: %i -> cluster = %i, (%f, %f), size: %f, dynamic: %i", nb_legs_detected,
                                                                                          c_id,
                                                                                          leg_detected[c_id].x,
                                                                                          leg_detected[c_id].y,
                                                                                          cluster_distance[c_id],
                                                                                          cluster_dynamic[c_id]);
        for(int loop2=0; loop2<nb_beams; loop2++)
          if ( cluster[loop2] == c_id ) {
            // dynamic legs are yellow
            display[nb_pts] = current_scan[loop2];

            colors[nb_pts].r = 1;
            colors[nb_pts].g = 1;
            colors[nb_pts].b = 0;
            colors[nb_pts].a = 1.0;

            nb_pts++;
          }
      }
      else
      {
        ROS_INFO("static leg found: %i -> cluster = %i, (%f, %f), size: %f, dynamic: %i", nb_legs_detected,
                                                                                          c_id,
                                                                                          leg_detected[c_id].x,
                                                                                          leg_detected[c_id].y,
                                                                                          cluster_distance[c_id],
                                                                                          cluster_dynamic[c_id]);
        for(int loop2=0; loop2<nb_beams; loop2++)
          if ( cluster[loop2] == c_id ) {
            // static legs are white
            display[nb_pts] = current_scan[loop2];

            colors[nb_pts].r = 1;
            colors[nb_pts].g = 1;
            colors[nb_pts].b = 1;
            colors[nb_pts].a = 1.0;

            nb_pts++;
          }
      }
      nb_pts++;
    }

  if (nb_legs_detected )
    ROS_INFO("%d legs have been detected.\n", nb_legs_detected);

}

}

