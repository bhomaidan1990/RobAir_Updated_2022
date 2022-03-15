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
#include <follow_me/simple_motion_detection.h>
#include <follow_me/simple_clustering.h>

namespace robair{

class Detection {
public:
    /**
     * \brief Default Class Constructor.
     */
    Detection(ros::NodeHandle& nh);

    /**
     * \brief Initialization.
     */
    bool init(ros::NodeHandle& nh);
    
    /**
     * \brief Laser Data Processing.
     */
    void update();

    /**
     * \brief Motion Detection.
     */
    void detectMotion();

    /**
     * \brief Visualize Motion Detection.
     */
    void visualizeMotion();

    /**
     * \brief Perform Clustering.
     */
    void simpleClustering();

    /**
     * \brief Visualize Clustering.
     */
    void visualizeClustering();


private:
    /**
     * \brief Node Handler.
     */
    ros::NodeHandle nh_;

    /**
     * \brief Detection Marker Publisher.
     */      
    ros::Publisher pub_detection_marker_;

    /**
     * \brief Detection Node Publisher.
     */  
    ros::Publisher pub_detection_node_;

    /**
     * \brief Motion Detection Object.
     */      
    SimpleMotionDetection motion_detector_;

    /**
     * \brief TODO.
     */  
    bool dynamic[1000];

    /**
     * \brief Clustering Object.
     */      
    SimpleClustering simple_clustering_;

    /**
     * \brief Num of Clusters.
     */  
    int nb_clusters;

    /**
     * \brief Laser Scans Cluster ID.
     */      
    int cluster[1000]; 

    /**
     * \brief Cluster Sizes.
     */      
    float cluster_distance[1000];

    /**
     * \brief Middle of Each Cluster.
     */      
    geometry_msgs::Point cluster_middle[1000];

    /**
     * \brief Dynamic Clusters.
     */      
    bool cluster_dynamic[1000];

    /**
     * \brief Max Num of Points to Parse.
     */  
    int max_pts;
};
}

#endif