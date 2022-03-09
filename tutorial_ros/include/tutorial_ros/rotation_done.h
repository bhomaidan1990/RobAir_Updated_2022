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

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/Point.h"

namespace robair{

class RotationDone {
public:
    /**
     * \brief Default Class Constructor.
     */
    RotationDone(ros::NodeHandle& nh);

    /**
     * \brief Default Class Destructor.
     */
    virtual ~RotationDone() {}

    /**
     * \brief Laser Data Processing.
     */
    void update();

    /**
     * \brief Odometry Callback.
     *
     */
    void odomCallback(const nav_msgs::Odometry::ConstPtr &o);

private:
    /**
     * \brief Node Handler.
     */
    ros::NodeHandle nh_;
    /**
     * \brief Odometry Subscriber for Communication.
     */
    ros::Subscriber sub_odometry_;
    /**
     * \brief TODO.
     */
    float rotation_done;
    /**
     * \brief Initial Orientation (ie, before starting the pid for rotation control).
     */
    float initial_orientation;
    /**
     * \brief Current Orientation (provided by the odometer).
     */
    float current_orientation;
    /**
     * \brief TODO.
     */
    bool init_odom;
    /**
     * \brief TODO.
     */
    bool first;
};
}