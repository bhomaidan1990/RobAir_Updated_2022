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

#include <follow_me/simple_motion_detection.h>

namespace robair{
/***********************************************************************************************************************
 * Class definitions: Simple Motion Detection
 */

/***********************************************************
 * Primary methods
 */
bool SimpleMotionDetection::init(int &num_beams, float (& r_)[1000], bool curr_robot_moving, bool prev_robot_moving){
    
    nb_beams = num_beams;
    current_robot_moving = curr_robot_moving;
    previous_robot_moving = prev_robot_moving;
    std::copy(std::begin(r_), std::end(r_), std::begin(r));

  return true;
}

void SimpleMotionDetection::run(){
  if (!current_robot_moving)
  {
    resetMotion();
    // if the robot is not moving then we can perform moving person detection
    if (stored_background)
      detectMotion();
    // DO NOT FORGET to store the background but when ???
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
}

void SimpleMotionDetection::detectMotion(){
  ROS_INFO("detecting motion");

  //loop over all the hits
  for (int loop=0; loop<nb_beams; loop++ )
  {
    // the detect of motion ONLY takes place when the robot is not moving, ie when current_robot_moving is false
    // when current_robot_moving is true, dynamic[loop] is false for all the beams

    // if the difference between ( the background and the current range ) is higher than "DETECTION_THRESHOLD"
    // then
    if( std::fabs(background[loop]-r[loop]) > DETECTION_THRESHOLD)
      dynamic[loop] = true;//the current hit is dynamic
    else
      dynamic[loop] = false;//else its static   
  }
}

void SimpleMotionDetection::storeBackground(){
  // store all the hits of the laser in the background table
  ROS_INFO("storing background");

  for (int loop = 0; loop < nb_beams; loop++)
      background[loop] = r[loop];
  stored_background = true;
  ROS_INFO("background stored");
}

void SimpleMotionDetection::resetMotion(){
  // for each hit, compare the current range with the background to detect motion
  ROS_INFO("reset motion");
  for (int loop = 0; loop < nb_beams; loop++)
      dynamic[loop] = false;
  ROS_INFO("reset_motion done");
}

}