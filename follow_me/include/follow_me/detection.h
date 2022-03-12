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

#ifndef ROBAIR_ROBOT_DETECTION_H
#define ROBAIR_ROBOT_DETECTION_H

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"

namespace robair{

class Detection {
public:
    /**
     * \brief Default Class Constructor.
     */
    Detection(ros::NodeHandle& nh);

    // /**
    //  * \brief Initialization.
    //  */
    // bool init(ros::NodeHandle& nh);
    
    /**
     * \brief Laser Data Processing.
     */
    void update();
    
    /**
     * \brief Store Background.
     */  
    void storeBackground();

    /**
     * \brief TODO.
     */  
    void resetMotion();

    /**
     * \brief Motion Detection.
     */  
    void detectMotion();

    /**
     * \brief TODO.
     */  
    void performClustering();

    /**
     * \brief TODO.
     */  
    void detectLegs();

    /**
     * \brief TODO.
     */  
    void detectPersons();

    /**
     * \brief TODO.
     */  
    void detectMovingPerson();

private : 
    /**
     * \brief Node Handler.
     */
    ros::NodeHandle nh_;

    /**
     * \brief TODO.
     */  
    bool stored_background;

    /**
     * \brief TODO.
     */  
    float background[1000];

    /**
     * \brief TODO.
     */  
    bool dynamic[1000];

    /**
     * \brief TODO.
     */  
    bool previous_robot_moving;

    /**
     * \brief Num of Clusters.
     */  
    int nb_pts;

    /**
     * \brief Num of Clusters.
     */  
    int nb_cluster;

    /**
     * \brief Laser Scans Cluster ID.
     */      
    int cluster[1000]; 

    /**
     * \brief Cluster Sizes.
     */      
    float cluster_size[1000];

    /**
     * \brief Middle of Each Cluster.
     */      
    geometry_msgs::Point cluster_middle[1000];

    /**
     * \brief Percentage of Dynamic Cluster.
     */      
    int cluster_dynamic[1000];

    /**
     * \brief Num of Detected Legs.
     */  
    int nb_legs_detected;

    /**
     * \brief Detected Legs Coordinates.
     */  
    geometry_msgs::Point leg_detected[1000];

    /**
     * \brief Leg Cluster.
     */  
    int leg_cluster[1000];

    /**
     * \brief Leg Dynamic.
     */  
    bool leg_dynamic[1000];

    /**
     * \brief Num of detected Persons.
     */  
    int nb_persons_detected;

    /**
     * \brief Detected Person Coordinates.
     */  
    geometry_msgs::Point person_detected[1000];

    /**
     * \brief Person dynamic.
     */  
    bool person_dynamic[1000];

    /**
     * \brief Detected Moving Person Coordinates.
     */  
    geometry_msgs::Point moving_person_detected;

    /**
     * \brief Laser Points to Display.
     */
    geometry_msgs::Point display[1000];

    /**
     * \brief Color Messages.
     */
    std_msgs::ColorRGBA colors[1000];

    /**
     * \brief Detection Node Publisher.
     */  
    ros::Publisher pub_detection_node_;

    /**
     * \brief Detection Marker Publisher.
     */      
    ros::Publisher pub_detection_marker_;
};

}

#endif