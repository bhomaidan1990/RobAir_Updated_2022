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

//threshold for clustering
#define CLUSTER_THRESHOLD 0.2
//to decide if a cluster is static or dynamic
#define DYNAMIC_THRESHOLD 75 

#include <follow_me/simple_clustering.h>

namespace robair{
/***********************************************************************************************************************
 * Class definitions: Simple Clustering
 */

/***********************************************************
 * Primary methods
 */

SimpleClustering::SimpleClustering(){
    // Initialization
    // init(geometry_msgs::Point (&current_scan)[1000]);
}

bool SimpleClustering::init(int &num_beams, geometry_msgs::Point (&current_scan)[1000], bool (&dynamic)[1000]){
    // To Do Some Checks and Initializations.
    nb_cluster = 0;
    nb_beams = num_beams;
    std::copy(std::begin(current_scan), std::end(current_scan), std::begin(scan_pts));
    std::copy(std::begin(dynamic), std::end(dynamic), std::begin(dynamic_));

    return true;
}

void SimpleClustering::performClustering(){

    //initialization of the first cluster
    int start = 0;// the first hit is the start of the first cluster
    int end;
    int dynamic_pts_percentage = 0;

    // loop over all the hits
    for( int loop=1; loop<nb_beams; loop++ ){
        // if EUCLIDIAN DISTANCE between (the previous hit and the current one) is higher than "CLUSTER_THRESHOLD"
        if(distancePoints(scan_pts[loop], scan_pts[loop-1]) > CLUSTER_THRESHOLD)
        {
        //the current hit doesnt belong to the same hit
        cluster[loop-1] = nb_cluster;
        
        dynamic_pts_percentage = 100 * dynamic_pts_percentage / (end - start);

        // 1/ we end the current cluster, so we update:
        // - end to store the last hit of the current cluster
        end = loop-1;

        // - cluster_distance to store the size of the cluster ie, the euclidian distance between the first hit of the cluster and the last one
        cluster_distance[nb_cluster] = distancePoints(scan_pts[start], scan_pts[end]);

        // - cluster_middle to store the middle of the cluster
        cluster_middle[nb_cluster] = scan_pts[(start + end)/2];
        
        // - cluster_dynamic to store cluster status dynamic/static
        if(dynamic_pts_percentage>DYNAMIC_THRESHOLD)
            cluster_dynamic[nb_cluster] = true; // dynamic
        else
            cluster_dynamic[nb_cluster] = false; // static
        
        // reset for next cluster
        dynamic_pts_percentage = 0; 
        // 2/ we start a new cluster with the current hit
        nb_cluster++;
        start = loop;
        }
        else
        {
            cluster[loop] = nb_cluster;
            if (dynamic_[loop])
                dynamic_pts_percentage++;
        }
    }
}


float SimpleClustering::distancePoints(geometry_msgs::Point &pa, geometry_msgs::Point &pb) {

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));
}
}