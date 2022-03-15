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

//used for detection of leg
#define LEG_SIZE_MIN 0.05
#define LEG_SIZE_MAX 0.25

#include <follow_me/detect_legs.h>

namespace robair{
/***********************************************************************************************************************
 * Class definitions: Simple Motion Detection
 */

/***********************************************************
 * Primary methods
 */
DetectLegs::DetectLegs(){

}

bool DetectLegs::init(int &num_clusters,  float (&cluster_distance)[1000], 
              geometry_msgs::Point (&cluster_middle)[1000], bool (&cluster_dynamic)[1000]){
    // Initialization
    nb_clusters = num_clusters;
    std::copy(std::begin(cluster_distance), std::end(cluster_distance), std::begin(cluster_distance_));
    std::copy(std::begin(cluster_middle), std::end(cluster_middle), std::begin(cluster_middle_));
    std::copy(std::begin(cluster_dynamic), std::end(cluster_dynamic), std::begin(cluster_dynamic_));

    return true;
}

void DetectLegs::detectLegs(){
    nb_legs_detected = 0;

    for (int c_id = 0; c_id < nb_clusters; c_id++){
        
        if(cluster_distance_[c_id]>LEG_SIZE_MIN && cluster_distance_[c_id]<LEG_SIZE_MAX){
            // Leg Detected
            nb_legs_detected++;
            // Store the Middle Point
            leg_detected[c_id] = cluster_middle_[c_id];
            // Store Dynamic status
            leg_dynamic[c_id] = cluster_dynamic_[c_id];
        }
    }
}
}