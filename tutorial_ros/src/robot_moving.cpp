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
#include <tutorial_ros/robot_moving.h>

int nb_static = 5;

namespace robair{
/***********************************************************************************************************************
 * Class definitions: RobotMoving
 */

/***********************************************************
 * Primary methods
 */
RobotMoving::RobotMoving(ros::NodeHandle& nh):
  nh_(nh) 
  {
    init(nh_);

    ros::Rate r(20);//this node is updated at 20hz

    while (ros::ok()) {
        ros::spinOnce();
        update();
        r.sleep();
    }

}

bool RobotMoving::init(ros::NodeHandle& nh){
    // To Do Some Checks and Initializations.
    ROS_INFO("Robot Moving Node Initialization...");

    // name_space_ = "/follow_me";
    // communication with person_detector
    pub_robot_moving_ = nh.advertise<std_msgs::Bool>("robot_moving", 1);   

    // communication with odometry
    sub_odometry_ = nh.subscribe("/odom", 1, &odomCallback);

    moving = 1;

    count = 0;

    not_moving_position.x = 0;
    not_moving_position.y = 0;

    not_moving_orientation = 0;
    
    init_odom = false;

    return true;
}

void RobotMoving::update() {

    if ( init_odom ) {//we wait for new data of odometry
        init_odom = false;
        if ( ( not_moving_position.x == position.x ) && ( not_moving_position.y == position.y ) && ( not_moving_orientation == orientation ) ) {
            count++;
            if ( ( count == nb_static ) && ( moving ) ) {
                ROS_INFO("robot is not moving");
                moving = false;
            }
        }
        else {
            not_moving_position.x = position.x;
            not_moving_position.y = position.y;
            not_moving_orientation = orientation;
            count = 0;
            if ( !moving ) {
                ROS_INFO("robot is moving");
                moving = true;
            }
        }

        std_msgs::Bool robot_moving_msg;
        robot_moving_msg.data = moving;

        pub_robot_moving_.publish(robot_moving_msg);
    }

}
}