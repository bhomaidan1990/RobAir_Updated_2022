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

#define LEGS_DISTANCE_MIN 0.1
#define LEGS_DISTANCE_MAX 0.7

#include <follow_me/detect_moving_person.h>
#include "ros/ros.h"

namespace robair{
/***********************************************************************************************************************
 * Class definitions: Simple Motion Detection
 */

/***********************************************************
 * Primary methods
 */

bool DetectMovinPerson::init(){
    
    return true;
}

void DetectMovinPerson::detectPerson(){
// a person has two legs located at less than "legs_distance_max" one from the other
// a moving person (ie, person_dynamic array) has 2 legs that are dynamic

    ROS_INFO("detecting persons");
    
    nb_persons_detected = 0;

    for (int loop_leg1=0; loop_leg1<nb_legs_detected; loop_leg1++){//loop over all the legs
        for (int loop_leg2=loop_leg1+1; loop_leg2<nb_legs_detected; loop_leg2++){//loop over all the legs
            // if the distance between two legs is lower than "legs_distance_max"
            // then we find a person

            // we update the person_detected table to store the middle of the person
            // we update the person_dynamic table to know if the person is moving or not

            nb_persons_detected++;
        }
    }

    if ( nb_persons_detected ) {
        ROS_INFO("%d persons have been detected.\n", nb_persons_detected);
    }
}

void DetectMovinPerson::movingPerson(){
   for (int loop=0; loop<nb_persons_detected; loop++)
        if ( person_dynamic[loop] )
        {

            //we update moving_person_tracked and publish it
            moving_person_detected = ...
            // pub_detection_node.publish(moving_person_detected);

        }
    ROS_INFO("detecting a moving person done");
}


float DetectMovinPerson::distancePoints(geometry_msgs::Point &pa, geometry_msgs::Point &pb) {

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));
}

}