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
    
    motion_detector_.setStoredBackground(false);

    return true;
}

void Detection::update()
{
  detectMotion();
  simpleClustering();
}

void Detection::detectMotion(){
  // we wait for new data of the laser and of the robot_moving_node to perform laser processing
  if (new_laser && init_robot)
  {
    ROS_INFO("\n");
    ROS_INFO("New data of laser received");
    ROS_INFO("New data of robot_moving received");

    motion_detector_.init(nb_beams, r, current_robot_moving, previous_robot_moving);
    motion_detector_.run();
    for (int loop = 0; loop < nb_beams; loop++)
      dynamic_[loop] = motion_detector_.getDynamicrArr()[loop];
    visualizeMotion();
    }
  else if (!init_robot)
    ROS_WARN("waiting for robot_moving_node");
}

void Detection::visualizeMotion(){
    // graphical display of the results
    int nb_pts = 0;
    for (int loop = 0; loop < nb_beams; loop++){
      if(dynamic_[loop]){
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
    if (nb_pts>0)
      ROS_INFO("nb_pts %i", nb_pts);
      populateMarkerTopic(pub_detection_marker_, nb_pts, display, colors);
}

void Detection::simpleClustering(){

  simple_clustering_.init(nb_beams, current_scan, dynamic_);
  ROS_INFO("performing clustering");
  simple_clustering_.performClustering();
  // visualizeClustering();
}

// void Detection::visualizeClustering(){

// }
}
