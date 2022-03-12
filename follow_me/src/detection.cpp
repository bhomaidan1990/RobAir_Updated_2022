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

//threshold for motion detection
#define DETECTION_THRESHOLD 0.2

//threshold for clustering
#define CLUSTER_THRESHOLD 0.2
//to decide if a cluster is static or dynamic
#define DYNAMIC_THRESHOLD 75 

//used for detection of leg
#define LEG_SIZE_MIN 0.05
#define LEG_SIZE_MAX 0.25
#define LEGS_DISTANCE_MIN 0.1
#define LEGS_DISTANCE_MAX 0.7

//used for uncertainty of leg
#define UNCERTAINTY_MIN_LEG 0.5
#define UNCERTAINTY_MAX_LEG 1
#define UNCERTAINTY_INC_LEG 0.05

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
  // init(nh_);
  // name_space_ = "/follow_me";
  sub_scan_ = nh_.subscribe("/scan", 1, &scanCallback);
  sub_robot_moving_ = nh_.subscribe("/tutorial_ros/robot_moving", 1, &robotMovingCallback);

  // communication with action
  pub_detection_node_ = nh_.advertise<geometry_msgs::Point>("goal_to_reach", 1);     // Preparing a topic to publish the goal to reach.
  pub_detection_marker_ = nh_.advertise<visualization_msgs::Marker>("detection_marker", 1); // Preparing a topic to publish our results. This will be used by the visualization tool rviz

  new_laser = false;
  init_robot = false;
  previous_robot_moving = true;
  stored_background = false;

  ros::Rate r(10);

  while (ros::ok()) {
      ros::spinOnce();
      update();
      r.sleep();
  }
}

void Detection::update()
{
  // we wait for new data of the laser and of the robot_moving_node to perform laser processing
  if (new_laser && init_robot)
  {
    ROS_INFO("\n");
    ROS_INFO("New data of laser received");
    ROS_INFO("New data of robot_moving received");

    nb_pts = 0;

    if (!current_robot_moving)
    { 
      //if the robot is not moving then we can perform moving person detection
      if (stored_background)
          detectMotion();
      // DO NOT FORGET to store the background but when ???
      if (previous_robot_moving)
          ROS_INFO("2");
          storeBackground();
      ROS_INFO("robot is not moving");
    }
    else
    {
      // IMPOSSIBLE TO DETECT MOTIONS because the base is moving
      // what is the value of dynamic table for each hit of the laser ?
      resetMotion();
      ROS_INFO("robot is moving");
    }
    previous_robot_moving = current_robot_moving;

    // we search for moving person in 4 steps
    performClustering(); // to perform clustering
    detectLegs();        // to detect moving legs using cluster
    detectPersons();     // to detect moving_person using moving legs detected
    detectMovingPerson();

    // graphical display of the results
    if (nb_pts>0)
      populateMarkerTopic(pub_detection_marker_, nb_pts, display, colors);
  }
  else if (!init_robot)
    ROS_WARN("waiting for robot_moving_node");
}

void Detection::storeBackground()
{
  // store all the hits of the laser in the background table
  ROS_INFO("storing background");

  for (int loop = 0; loop < nb_beams; loop++)
      background[loop] = r[loop];
  stored_background = true;
  ROS_INFO("background stored");
}

void Detection::resetMotion()
{
  // for each hit, compare the current range with the background to detect motion
  ROS_INFO("reset motion");
  for (int loop = 0; loop < nb_beams; loop++)
      dynamic[loop] = false;

  ROS_INFO("reset_motion done");
}

void Detection::detectMotion()
{
  ROS_INFO("detecting motion");

  // nb_pts = 0;

  for (int loop=0; loop<nb_beams; loop++ )
  {//loop over all the hits
    // the detect of motion ONLY takes place when the robot is not moving, ie when current_robot_moving is false
    // when current_robot_moving is true, dynamic[loop] is false for all the beams

    // if the difference between ( the background and the current range ) is higher than "DETECTION_THRESHOLD"
    // then
    if( (background[loop]-r[loop]) > DETECTION_THRESHOLD)
      dynamic[loop] = true;//the current hit is dynamic
    else
      dynamic[loop] = false;//else its static

    if ( dynamic[loop] ) {

      //display in blue of hits that are dynamic
      display[nb_pts] = current_scan[loop];

      colors[nb_pts].r = 0;
      colors[nb_pts].g = 0;
      colors[nb_pts].b = 1;
      colors[nb_pts].a = 1.0;

      nb_pts++;
    }
  }

  ROS_INFO("motion detected");
}

void Detection::performClustering()
{
  // store in the table cluster, the cluster of each hit of the laser
  // if the distance between the previous hit and the current one is higher than "CLUSTER_THRESHOLD"
  // then we end the current cluster with the previous hit and start a new cluster with the current hit
  // else the current hit belongs to the current cluster

  ROS_INFO("performing clustering");

  nb_cluster = 0;//to count the number of cluster

  //initialization of the first cluster
  int start = 0;// the first hit is the start of the first cluster
  int end;
  int nb_dynamic = 0;// to count the number of hits of the current cluster that are dynamic

  // nb_pts = 0;
  
  //graphical display of the start of the current cluster in green
  display[nb_pts] = current_scan[start];

  colors[nb_pts].r = 0;
  colors[nb_pts].g = 1;
  colors[nb_pts].b = 0;
  colors[nb_pts].a = 1.0;
  nb_pts++;

  for( int loop=1; loop<nb_beams; loop++ )//loop over all the hits
  {
    // if EUCLIDIAN DISTANCE between (the previous hit and the current one) is higher than "CLUSTER_THRESHOLD"
    if(distancePoints(current_scan[loop], current_scan[loop-1]) > CLUSTER_THRESHOLD)
    {
      //the current hit doesnt belong to the same hit
      cluster[loop-1] = nb_cluster;

      nb_dynamic = 100 * nb_dynamic / (end - start);

      // 1/ we end the current cluster, so we update:
      // - end to store the last hit of the current cluster
      end = loop-1;
      // - cluster_size to store the size of the cluster ie, the euclidian distance between the first hit of the cluster and the last one
      cluster_size[nb_cluster] = distancePoints(current_scan[start], current_scan[end]);
      // - cluster_middle to store the middle of the cluster
      cluster_middle[nb_cluster] = current_scan[(start + end)/2];
      // - cluster_dynamic to store the percentage of hits of the current cluster that are dynamic
      cluster_dynamic[nb_cluster] = nb_dynamic;

      // graphical display of the end of the current cluster in red
      display[nb_pts] = current_scan[end];

      colors[nb_pts].r = 1;
      colors[nb_pts].g = 0;
      colors[nb_pts].b = 0;
      colors[nb_pts].a = 1.0;
      nb_pts++;

      //textual display
      ROS_INFO("cluster[%i] = (%f, %f): hit[%i](%f, %f) -> hit[%i](%f, %f), size: %f, dynamic: %i, %i", nb_cluster,
                                                                          cluster_middle[nb_cluster].x,
                                                                          cluster_middle[nb_cluster].y,
                                                                          start,
                                                                          current_scan[start].x,
                                                                          current_scan[start].y,
                                                                          end,
                                                                          current_scan[end].x,
                                                                          current_scan[end].y,
                                                                          cluster_size[nb_cluster],
                                                                          cluster_dynamic[nb_cluster],
                                                                          cluster_dynamic[nb_cluster]);

      //graphical display of the middle of the current cluster in blue
      display[nb_pts] = cluster_middle[nb_cluster];

      colors[nb_pts].r = 0;
      colors[nb_pts].g = 0;
      colors[nb_pts].b = 1;
      colors[nb_pts].a = 1.0;
      nb_pts++;

      // 2/ we start a new cluster with the current hit
      nb_cluster++;
      start = loop;
      nb_dynamic = 0;// to count the number of hits of the current cluster that are dynamic

      //graphical display of the start of the current cluster in green
      display[nb_pts] = current_scan[start];

      colors[nb_pts].r = 0;
      colors[nb_pts].g = 1;
      colors[nb_pts].b = 0;
      colors[nb_pts].a = 1.0;
      nb_pts++;

    }
    else
    {
        cluster[loop] = nb_cluster;

        // nb_dynamic = ? ? ? ;
        if (dynamic[loop])
          nb_dynamic++;
    }

  // Dont forget to update the different information for the last cluster
  //...

  ROS_INFO("clustering performed");

  }
}

void Detection::detectLegs()
{
  // a leg is a cluster:
  // - with a size higher than "leg_size_min";
  // - with a size lower than "leg_size_max;
  // if more than "DYNAMIC_THRESHOLD"% of its hits are dynamic the leg is considered to be dynamic

  ROS_INFO("detecting legs");
  nb_legs_detected = 0;

  nb_pts = 0;
  for (int loop=0; loop<nb_cluster; loop++)//loop over all the clusters
    // if the size of the current cluster is higher than "leg_size_min" and lower than "leg_size_max"
    // then the current cluster is a leg
    if(cluster_size[loop]>LEG_SIZE_MIN && cluster_size[loop]<LEG_SIZE_MAX){
      // // we update:
      // - the leg_detected table to store the middle of the moving leg;
      leg_detected[loop] = cluster_middle[loop];
      // - the leg_cluster to store the cluster corresponding to a leg;
      leg_cluster[loop] = loop;
      // - the leg_dynamic to know if the leg is dynamic or not.
      if (cluster_dynamic[loop]> DYNAMIC_THRESHOLD)
        leg_dynamic[loop] = true;
      else
        leg_dynamic[loop] = false;

      if (leg_dynamic[nb_legs_detected])
      {
        ROS_INFO("moving leg found: %i -> cluster = %i, (%f, %f), size: %f, dynamic: %i", nb_legs_detected,
                                                                                          loop,
                                                                                          leg_detected[nb_legs_detected].x,
                                                                                          leg_detected[nb_legs_detected].y,
                                                                                          cluster_size[loop],
                                                                                          cluster_dynamic[loop]);
        for(int loop2=0; loop2<nb_beams; loop2++)
          if ( cluster[loop2] == loop ) {

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
                                                                                          loop,
                                                                                          leg_detected[nb_legs_detected].x,
                                                                                          leg_detected[nb_legs_detected].y,
                                                                                          cluster_size[loop],
                                                                                          cluster_dynamic[loop]);
        for(int loop2=0; loop2<nb_beams; loop2++){
          if ( cluster[loop2] == loop ) {

              // static legs are white
              display[nb_pts] = current_scan[loop2];

              colors[nb_pts].r = 1;
              colors[nb_pts].g = 1;
              colors[nb_pts].b = 1;
              colors[nb_pts].a = 1.0;

              nb_pts++;
            }
        }

      nb_legs_detected++;
      }
    // nb_pts++;
    }

  if ( nb_legs_detected )
      ROS_INFO("%d legs have been detected.\n", nb_legs_detected);

  ROS_INFO("detecting legs done");
}

void Detection::detectPersons()
{
  // a person has two legs located at less than "legs_distance_max" one from the other
  // a moving person (ie, person_dynamic array) has 2 legs that are dynamic

  ROS_INFO("detecting persons");
  nb_persons_detected = 0;

  for (int loop_leg1=0; loop_leg1<nb_legs_detected; loop_leg1++)//loop over all the legs
      for (int loop_leg2=loop_leg1+1; loop_leg2<nb_legs_detected; loop_leg2++)//loop over all the legs
          // if the distance between two legs is lower than "legs_distance_max"
          if(distancePoints(leg_detected[loop_leg1], leg_detected[loop_leg2])<LEGS_DISTANCE_MAX){
          // then we find a person
        
          // we update the person_detected table to store the middle of the person
          // we update the person_dynamic table to know if the person is moving or not
          if ( person_dynamic[nb_persons_detected] )
          {
              ROS_INFO("moving person detected: leg[%i]+leg[%i] -> (%f, %f)", loop_leg1,
                                                                              loop_leg2,
                                                                              person_detected[nb_persons_detected].x,
                                                                              person_detected[nb_persons_detected].y);
              // a moving person detected is green
              display[nb_pts] = person_detected[nb_persons_detected];

              colors[nb_pts].r = 0;
              colors[nb_pts].g = 1;
              colors[nb_pts].b = 0;
              colors[nb_pts].a = 1.0;

              nb_pts++;
          }
          else
          {
              ROS_INFO("static person detected: leg[%i]+leg[%i] -> (%f, %f)", loop_leg1,
                                                                              loop_leg2,
                                                                              person_detected[nb_persons_detected].x,
                                                                              person_detected[nb_persons_detected].y);
              // a static person detected is red
              display[nb_pts] = person_detected[nb_persons_detected];

              colors[nb_pts].r = 1;
              colors[nb_pts].g = 0;
              colors[nb_pts].b = 0;
              colors[nb_pts].a = 1.0;

              nb_pts++;
          }

          nb_persons_detected++;
      }

  if ( nb_persons_detected ) {
      ROS_INFO("%d persons have been detected.\n", nb_persons_detected);
  }

  ROS_INFO("persons detected");
}

void Detection::detectMovingPerson() 
{

  ROS_INFO("detecting a moving person");

  for (int loop=0; loop<nb_persons_detected; loop++)
  {
    if ( person_dynamic[loop] )
    {
      //we update moving_person_tracked and publish it
      moving_person_detected = person_detected[loop];
      pub_detection_node_.publish(moving_person_detected);
    }
  ROS_INFO("detecting a moving person done");
  }
}

}