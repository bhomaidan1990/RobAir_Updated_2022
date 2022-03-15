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

#ifndef ROBAIR_SIMPLE_CLUSTERING_H
#define ROBAIR_SIMPLE_CLUSTERING_H

#include "geometry_msgs/Point.h"

namespace robair{

class SimpleClustering{

public:
    /**
     * \brief Default Class Constructor.
     */
    SimpleClustering();

    /**
     * \brief Initialization.
     */
    bool init(int &num_beams, geometry_msgs::Point (&current_scan)[1000], bool (&dynamic)[1000]);

    /**
     * \brief Clustering Function.
     */
    void performClustering();

    /**
     * \brief Eucledian Distance Between Two Points.
     *
     * \return float Eucledian Distance in Meters.
     */
    float distancePoints(geometry_msgs::Point &pa, geometry_msgs::Point &pb);

    //------------------------------------
    // Getter Functions 
    //------------------------------------
    // Num of Clusters Getter
    const int getNumClusters(){
        return nb_cluster;
    }
   /**
     * \brief To Get the Middle Point of a Cluster.
    * \return Static geometry_msgs::Point Array cluster_middle
    */ 
    const geometry_msgs::Point *getClusterMiddle() const{
        return cluster_middle;
    }

   /**
     * \brief To Get the Cluster Size
    * \return Static int Array cluster_size
    */  
    const int *getClusterArr() const{
        return cluster;
    }

    /**
     * \brief To Get the Cluster Euclidean Distance
    * \return Static int Array cluster_size
    */ 
    const float *getClusterSize() const{
        return cluster_size;
    }

    /**
     * \brief To Get the 100 * Percentage of Dynamic Points Inside a Cluster
    * \return Static Array cluster_dynamic
    */ 
    const int *getClusterDynamic() const{
        return cluster_dynamic;
    }

private:

    /**
     * \brief Number of Laser Scan Beams.
     */
    int nb_beams;

    /**
     * \brief Raw Laser Scan to Cluster.
     */      
    geometry_msgs::Point scan_pts[1000];

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
     * \brief TODO.
     */  
    bool dynamic_[1000];
};

}

#endif
