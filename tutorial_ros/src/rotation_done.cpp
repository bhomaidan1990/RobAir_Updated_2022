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

// Security Distance
#define SECURITY_DISTANCE 0.5

#include <tutorial_ros/rotation_done.h>

namespace robair{

RotationDone::RotationDone(ros::NodeHandle& nh):
  nh_(nh) 
  {
    // communication with odometry
    sub_odometry_ = nh_.subscribe("odom", 1, &RotationDone::odomCallback, this);

    init_odom = false;
    first = true;
    rotation_done = 0;

    //INFINITE LOOP TO COLLECT LASER DATA AND PROCESS THEM
    ros::Rate r(10);// this node will run at 10hz
    while (ros::ok()) {
        ros::spinOnce();//each callback is called once to collect new data: laser + robot_moving
        update();//processing of data
        r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }
}

void RotationDone::update() {

    if ( init_odom )
    {

//        if ( first )
        // initial_orientation = ...;
        first = false;

        //rotation_done = ...;

        //do not forget that rotation_done must always be between -M_PI and +M_PI
        if ( rotation_done > M_PI )
        {
            ROS_WARN("rotation_done > 180 degrees: %f degrees -> %f degrees", rotation_done*180/M_PI, (rotation_done-2*M_PI)*180/M_PI);
            rotation_done -= 2*M_PI;
        }
        else
            if ( rotation_done < -M_PI )
            {
                ROS_WARN("rotation_done < -180 degrees: %f degrees -> %f degrees", rotation_done*180/M_PI, (rotation_done+2*M_PI)*180/M_PI);
                rotation_done += 2*M_PI;
            }

        ROS_INFO("current_orientation: %f, initial_orientation: %f, rotation_done: %f", current_orientation*180/M_PI, initial_orientation*180/M_PI, rotation_done*180/M_PI);

    }

}

void RotationDone::odomCallback(const nav_msgs::Odometry::ConstPtr& o) {

    init_odom = true;
    current_orientation = tf::getYaw(o->pose.pose.orientation);

}
}


