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

#ifndef ROBAIR_DETECT_LEGS_H
#define ROBAIR_DETECT_LEGS_H

#include "geometry_msgs/Point.h"

namespace robair{

class DetectLegs{

public:
    /**
     * \brief Default Class Constructor.
     */
    DetectLegs();

    /**
     * \brief Initialization.
     */
    bool init(int &num_clusters, float (&cluster_distance)[1000], 
              geometry_msgs::Point (&cluster_middle)[1000], bool (&cluster_dynamic)[1000]);

    /**
     * \brief Initialization.
     */
    void detectLegs();
    //------------------------------------
    // Getter Functions 
    //------------------------------------
        // Num of Clusters Getter
    int getNumLegs(){
        return nb_legs_detected;
    }
    
    /**
     * \brief To Get the Cluster
    * \return Static int Array leg_cluster
    */  
    int *getLegCluster(){
        return leg_cluster;
    }

   /**
     * \brief To Get the Middle Point of a Leg Cluster.
    * \return Static geometry_msgs::Point Array leg_detected
    */ 
    geometry_msgs::Point *getLegMiddle(){
        return leg_detected;
    }

    /**
     * \brief To Get the Cluster
    * \return Static bool Array leg_dynamic
    */  
    bool *getLegDynamic(){
        return leg_dynamic;
    }

private:
    /**
     * \brief Num of Detected Legs.
     */
    int nb_legs_detected;

    /**
     * \brief Detected Legs Mid Points.
     */
    geometry_msgs::Point leg_detected[1000];

    /**
     * \brief Leg Cluster.
     */
    int leg_cluster[1000];

    /**
     * \brief Dynamic Leg (Moving).
     */
    bool leg_dynamic[1000];

    /**
     * \brief Num of Clusters.
     */  
    int nb_clusters;

    /**
     * \brief Laser Scans Cluster ID.
     */      
    int cluster_[1000];

    /**
     * \brief Cluster Sizes.
     */      
    float cluster_distance_[1000];

    /**
     * \brief Middle of Each Cluster.
     */      
    geometry_msgs::Point cluster_middle_[1000];

    /**
     * \brief Dynamic Clusters.
     */      
    bool cluster_dynamic_[1000];
};

}

#endif