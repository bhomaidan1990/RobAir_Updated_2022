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
#define DETECTION_THRESHOLD 0.2 //threshold for motion detection
#define DYNAMIC_THRESHOLD 75 //to decide if a cluster is static or dynamic

//threshold for clustering
#define CLUSTER_THRESHOLD 0.2

//used for detection of leg
#define LEG_SIZE_MIN 0.05
#define LEG_SIZE_MAX 0.25
#define LEG_DISTANCE_MAX 0.7

//used for frequency
#define FREQUENCY_INIT 5
#define FREQUENCY_MAX 25

//used for uncertainty associated to the tracked person
#define UNCERTAINTY_MIN 0.5
#define UNCERTAINTY_MAX 1
#define UNCERTAINTY_INC 0.05

#include <tutorial_ros/utility.h>
#include <follow_me/datmo.h>

namespace robair{
/***********************************************************************************************************************
 * Class definitions: DATMO
 */

/***********************************************************
 * Primary methods
 */

DATMO::DATMO(ros::NodeHandle& nh):
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

bool DATMO::init(ros::NodeHandle& nh){

    sub_scan_ = nh.subscribe("/scan", 1, &scanCallback);
    sub_robot_moving_ = nh.subscribe("/tutorial_ros/robot_moving", 1, &robotMovingCallback);

    // communication with action
    pub_datmo = nh.advertise<geometry_msgs::Point>("goal_to_reach", 1);     // Preparing a topic to publish the goal to reach.

    pub_datmo_marker = nh.advertise<visualization_msgs::Marker>("datmo_marker", 1); // Preparing a topic to publish our results. This will be used by the visualization tool rviz

    new_laser = false;
    init_robot = false;

    previous_robot_moving = true;
    stored_background = false;

    tracking_mode = false;

    return true;
}

void DATMO::update(){

    // we wait for new data of the laser and of the robot_moving_node to perform laser processing
    if ( new_laser && init_robot ) {

        ROS_INFO("\n");
        ROS_INFO("New data of laser received");
        ROS_INFO("New data of robot_moving received");

        nb_pts = 0;

        if ( !current_robot_moving ) {

            if(stored_background){
                detectMotion();
                /* if the robot is not moving then we can perform moving person detection
                we search for moving person in 4 steps */
                performClustering();//to perform clustering
                detectLegs();//to detect moving legs using cluster
                detectPersons();//to detect moving_person using moving legs detected
                detectMovingPerson();
                trackMovingPerson();
            }
            //DO NOT FORGET to store the background but when ???
            if(previous_robot_moving)
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

        //graphical display of the results
        populateMarkerTopic(pub_datmo_marker, nb_pts, display, colors);        
    }
    else
        if ( !init_robot )
            ROS_WARN("waiting for robot_moving_node");
}

void DATMO::storeBackground() {
// store all the hits of the laser in the background table

    ROS_INFO("storing background");

    for (int loop=0; loop<nb_beams; loop++)
        background[loop] = r[loop];
    stored_background = true;
    ROS_INFO("background stored");

}

void DATMO::resetMotion() {
// for each hit, compare the current range with the background to detect motion

    ROS_INFO("reset motion");
    for (int loop=0 ; loop<nb_beams; loop++ )
        dynamic[loop] = false;

    ROS_INFO("reset_motion done");

}

void DATMO::detectMotion() {

    ROS_INFO("detecting motion");

    nb_pts = 0;
    //loop over all the hits
    for (int loop=0; loop<nb_beams; loop++ )
    {
        /*the detect of motion ONLY takes place when the robot is not moving, ie when current_robot_moving is false
        when current_robot_moving is true, dynamic[loop] is false for all the beams*/

        /*if the difference between ( the background and the current range ) is higher than "DETECTION_THRESHOLD"
        then*/
        if(std::fabs(r[loop] - background[loop]) > DETECTION_THRESHOLD)
            dynamic[loop] = true;//the current hit is dynamic
        else
            dynamic[loop] = false;//else its static

    // if ( dynamic[loop] ) {

    //     //display in blue of hits that are dynamic
    //     display[nb_pts] = current_scan[loop];

    //     colors[nb_pts].r = 0;
    //     colors[nb_pts].g = 0;
    //     colors[nb_pts].b = 1;
    //     colors[nb_pts].a = 1.0;

    //     nb_pts++;
    //     }
    }

    ROS_INFO("motion detected");

}

void DATMO::performClustering() 
{
/*----------------------------------------------------------------
 * store in the table cluster, the cluster of each hit of the laser
 * if the distance between the previous hit and the current one is higher than "CLUSTER_THRESHOLD"
 * then we end the current cluster with the previous hit and start a new cluster with the current hit
 * else the current hit belongs to the current cluster
 ---------------------------------------------------------------*/

    ROS_INFO("performing clustering");

    nb_cluster = 0;//to count the number of cluster

    //initialization of the first cluster
    int start = 0;// the first hit is the start of the first cluster
    int end;
    int nb_dynamic = 0;// to count the number of hits of the current cluster that are dynamic

    nb_pts = 0;
    //graphical display of the start of the current cluster in green
    // display[nb_pts] = current_scan[start];

    // colors[nb_pts].r = 0;
    // colors[nb_pts].g = 1;
    // colors[nb_pts].b = 0;
    // colors[nb_pts].a = 1.0;
    // nb_pts++;

    //loop over all the hits
    for( int loop=1; loop<nb_beams; loop++ ){
        /* 
         * if EUCLIDIAN DISTANCE between (the previous hit and the current one) is higher than "CLUSTER_THRESHOLD"
         * the current hit doesnt belong to the same hit
         */
        if(distancePoints(current_scan[loop-1], current_scan[loop]) > CLUSTER_THRESHOLD){
            /* 1/ we end the current cluster, so we update:
            * - end to store the last hit of the current cluster*/
            end = loop-1;

            cluster[end] = nb_cluster;

            if(dynamic[end])
                nb_dynamic ++;
            
            /*- cluster_size to store the size of the cluster ie, the euclidian distance between the first hit of the cluster and the last one
            * - cluster_middle to store the middle of the cluster
            * - cluster_dynamic to store the percentage of hits of the current cluster that are dynamic 
            */
            cluster_size[nb_cluster] = distancePoints(current_scan[start], current_scan[end]);
            cluster_middle[nb_cluster] = current_scan[(start+end)/2];
            if((end-start>0))
                cluster_dynamic[nb_cluster] =  (100 * nb_dynamic) / (end - start + 1);
            else
                cluster_dynamic[nb_cluster] = 0;
            // graphical display of the end of the current cluster in red
            // display[nb_pts] = current_scan[end];

            // colors[nb_pts].r = 1;
            // colors[nb_pts].g = 0;
            // colors[nb_pts].b = 0;
            // colors[nb_pts].a = 1.0;
            // nb_pts++;

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

            /* 2/ we start a new cluster with the current hit */
            nb_cluster++;
            start = loop;
            nb_dynamic = 0;// to count the number of hits of the current cluster that are dynamic

            //graphical display of the start of the current cluster in green
            // display[nb_pts] = current_scan[start];

            // colors[nb_pts].r = 0;
            // colors[nb_pts].g = 1;
            // colors[nb_pts].b = 0;
            // colors[nb_pts].a = 1.0;
            // nb_pts++;
        }
        else
        {
            cluster[loop-1] = nb_cluster;
            if(dynamic[loop-1])
                nb_dynamic ++;
        }
    }
    //Dont forget to update the different information for the last cluster
    //...

    ROS_INFO("clustering performed");

}

void DATMO::detectLegs() 
{
/* a leg is a cluster:
// - with a size higher than "LEG_SIZE_MIN";
// - with a size lower than "LEG_SIZE_MAX;
// if more than "DYNAMIC_THRESHOLD"% of its hits are dynamic the leg is considered to be dynamic*/

    ROS_INFO("detecting legs");
    nb_legs_detected = 0;

    nb_pts = 0;
    //loop over all the clusters
    for (int loop=0; loop<nb_cluster; loop++){
        /*if the size of the current cluster is higher than "LEG_SIZE_MIN" and lower than "LEG_SIZE_MAX"
        then the current cluster is a leg*/
        if(cluster_size[loop]>LEG_SIZE_MIN && cluster_size[loop]<LEG_SIZE_MAX){

            /* we update:
            - the leg_detected table to store the middle of the moving leg;
            - the leg_cluster to store the cluster corresponding to a leg;
            - the leg_dynamic to know if the leg is dynamic or not.*/
            leg_detected[nb_legs_detected] = cluster_middle[loop];

            leg_cluster[nb_legs_detected] = cluster[loop];
            
            if(cluster_dynamic[loop]> DYNAMIC_THRESHOLD)
                leg_dynamic[nb_legs_detected] = true;
            else
                leg_dynamic[nb_legs_detected] = false;

            if ( leg_dynamic[nb_legs_detected] )
            {
                ROS_INFO("moving leg found: %i -> cluster = %i, (%f, %f), size: %f, dynamic: %i", nb_legs_detected,
                                                                                                  loop,
                                                                                                  leg_detected[nb_legs_detected].x,
                                                                                                  leg_detected[nb_legs_detected].y,
                                                                                                  cluster_size[loop],
                                                                                                  cluster_dynamic[loop]);
                // for(int loop2=0; loop2<nb_beams; loop2++){
                //     if ( cluster[loop2] == loop ) {

                //         // dynamic legs are yellow
                //         display[nb_pts] = current_scan[loop2];

                //         colors[nb_pts].r = 1;
                //         colors[nb_pts].g = 1;
                //         colors[nb_pts].b = 0;
                //         colors[nb_pts].a = 1.0;

                //         nb_pts++;
                //     }
                // }
            }
            else
            {
                ROS_INFO("static leg found: %i -> cluster = %i, (%f, %f), size: %f, dynamic: %i", nb_legs_detected,
                                                                                                  loop,
                                                                                                  leg_detected[nb_legs_detected].x,
                                                                                                  leg_detected[nb_legs_detected].y,
                                                                                                  cluster_size[loop],
                                                                                                  cluster_dynamic[loop]);
                // for(int loop2=0; loop2<nb_beams; loop2++)
                //     if ( cluster[loop2] == loop ) {

                //         // static legs are white
                //         display[nb_pts] = current_scan[loop2];

                //         colors[nb_pts].r = 1;
                //         colors[nb_pts].g = 1;
                //         colors[nb_pts].b = 1;
                //         colors[nb_pts].a = 1.0;

                //         nb_pts++;
                //     }
            }
        
            nb_legs_detected++;
        }
        nb_pts++;
    }

    if ( nb_legs_detected )
        ROS_INFO("%d legs have been detected.\n", nb_legs_detected);

    ROS_INFO("detecting legs done");

}

void DATMO::detectPersons() 
{
/* a person has two legs located at less than "LEG_DISTANCE_MAX" one from the other
// a moving person (ie, person_dynamic array) has 2 legs that are dynamic*/

    ROS_INFO("detecting persons");
    nb_persons_detected = 0;
    //loop over all the legs
    for (int loop_leg1=0; loop_leg1<nb_legs_detected; loop_leg1++){
        //loop over all the legs
        for (int loop_leg2=loop_leg1+1; loop_leg2<nb_legs_detected; loop_leg2++){
            /*if the distance between two legs is lower than "LEG_DISTANCE_MAX"
            then we find a person*/
            if(distancePoints(leg_detected[loop_leg1], leg_detected[loop_leg2])<LEG_DISTANCE_MAX)
            {

                /* we update the person_detected table to store the middle of the person
                // we update the person_dynamic table to know if the person is moving or not*/
                person_detected[loop_leg1].x = (leg_detected[loop_leg1].x + leg_detected[loop_leg2].x) / 2;
                person_detected[loop_leg1].y = (leg_detected[loop_leg1].y + leg_detected[loop_leg2].y) / 2;
                person_detected[loop_leg1].z = (leg_detected[loop_leg1].z + leg_detected[loop_leg2].z) / 2;

                person_dynamic[loop_leg1] = leg_dynamic[loop_leg1] && leg_dynamic[loop_leg2];

                if (person_dynamic[nb_persons_detected])
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
                    // display[nb_pts] = person_detected[nb_persons_detected];

                    // colors[nb_pts].r = 1;
                    // colors[nb_pts].g = 0;
                    // colors[nb_pts].b = 0;
                    // colors[nb_pts].a = 1.0;

                    // nb_pts++;
                }

                nb_persons_detected++;
            }
        }
    }
    if ( nb_persons_detected ) {
        ROS_INFO("%d persons have been detected.\n", nb_persons_detected);
    }

    ROS_INFO("persons detected");

}//detect_persons

void DATMO::detectMovingPerson() 
{
    ROS_INFO("detecting a moving person");

    for (int loop=0; loop<nb_persons_detected; loop++)
        if ( person_dynamic[loop] )
        {
            //we update moving_person_tracked and publish it
            moving_person_tracked = person_detected[loop];
            pub_datmo.publish(moving_person_tracked);
        }
    ROS_INFO("detecting a moving person done");
}

void DATMO::trackMovingPerson() {

    ROS_INFO("tracking a moving person");

    bool associated = false;
    float distance_min = UNCERTAINTY_MAX;
    int index_min;

    //we search for the closest detection to the tracking person
    for( int loop_detection=0; loop_detection<nb_persons_detected; loop_detection++ )
    {   
        //EUCLIDIAN DISTANCE between the moving_person_tracked and the current_person_detected
        float current_dist = distancePoints(moving_person_tracked, person_detected[loop_detection]);
        ROS_INFO("distance with [%i] = %f", loop_detection, current_dist);
        if (  current_dist < distance_min ) {
            // we store the info about the closest detection
            index_min = loop_detection;
            associated = true;
            ROS_INFO("track associated with %i", loop_detection);
        }
    }

    nb_pts -= nb_persons_detected;

    if ( associated )
    {
        // if the moving_person_tracked has been associated how we update moving_person_tracked, frequency and uncertainty
        pub_datmo.publish(moving_person_tracked);

        ROS_INFO("moving_person_tracked: (%f, %f), %i, %f", moving_person_tracked.x,
                                                            moving_person_tracked.y,
                                                            frequency,
                                                            uncertainty);
        // moving person tracked is green
        display[nb_pts] = moving_person_tracked;

        colors[nb_pts].r = 0;
        colors[nb_pts].g = 1;
        colors[nb_pts].b = 0;
        colors[nb_pts].a = 1.0;

        nb_pts++;
    }
    else {
        // if the moving_person_tracked has not been associated how we update moving_person_tracked, frequency and uncertainty
        ROS_INFO("moving_person_tracked: (%f, %f), %i, %f", moving_person_tracked.x,
                                                            moving_person_tracked.y,
                                                            frequency,
                                                            uncertainty);
        if(current_robot_moving)
            tracking_mode = false; /*when do we switch tracking_mode to false ???*/
        if ( !tracking_mode )
        {
            ROS_WARN("moving person tracked has been lost");
            moving_person_tracked.x = 0;
            moving_person_tracked.y = 0;
            pub_datmo.publish(moving_person_tracked);
        }

    }

    ROS_INFO("tracking of a moving person done");

}

}