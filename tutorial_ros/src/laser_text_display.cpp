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

#include <tutorial_ros/utility.h>
#include <tutorial_ros/laser_text_display.h>

namespace robair
{
/***********************************************************************************************************************
 * Class definitions: LaserTextDisplay
 */

/***********************************************************
 * Primary methods
 */

LaserTextDisplay::LaserTextDisplay(ros::NodeHandle& nh):
  nh_(nh) 
  {
      // Initializations.
      init(nh_);

      // INFINTE LOOP TO COLLECT LASER DATA AND PROCESS THEM
      ros::Rate r(10); // this node will run at 10hz
      while (ros::ok())
      {
          ros::spinOnce(); // each callback is called once to collect new data: laser
          update();        // processing of data
          // we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
          r.sleep();
    }

}

bool LaserTextDisplay::init(ros::NodeHandle& nh){
    // To Do Some Checks and Initializations.
    ROS_INFO("Laser Text Display Node Initialization...");
    // name_space_ = "/tutorial_ros";

    sub_scan_ = nh.subscribe("/scan", 1, &scanCallback);

    new_laser = false;

    return true;
}

void LaserTextDisplay::update() {
    // we wait for new data of the laser
    if ( new_laser )
    {
        ROS_INFO("New data of laser received");

        for ( int loop=0 ; loop < nb_beams; loop++ )
            ROS_INFO("r[%i] = %f, theta[%i] (in degrees) = %f, x[%i] = %f, y[%i] = %f", loop, 
              r[loop], loop, theta[loop]*180/M_PI, loop, current_scan[loop].x, loop, current_scan[loop].y);

        new_laser = false;
    }
}

}