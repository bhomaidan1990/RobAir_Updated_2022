#define ROTATION_ERROR_THRESHOLD 0.2//radians

#define KP_R 0.5
#define KI_R 0
#define KD_R 0

#include <tutorial_ros/utility.h>
#include <follow_me/rotation.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Quaternion.h"

namespace robair{
/***********************************************************************************************************************
 * Class definitions: Rotation
 */

/***********************************************************
 * Primary methods
 */

Rotation::Rotation(ros::NodeHandle& nh):
  nh_(nh) 
  {
    init(nh_);

    //INFINITE LOOP TO COLLECT LASER DATA AND PROCESS THEM
    ros::Rate r(10);// this node will run at 10hz
    while (ros::ok()) {
        ros::spinOnce();//each callback is called once to collect new data: laser + robot_moving
        update();//processing of data
        r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }

}

bool Rotation::init(ros::NodeHandle& nh){
    // communication with cmd_vel to command the mobile robot
    pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // communication with odometry
    sub_odometry_ = nh.subscribe("/odom", 1, &odomCallback);

    // communication with datmo
    sub_goal_to_reach = nh.subscribe("goal_to_reach", 1, &Rotation::goalToReachCallback, this);

    new_goal_to_reach = false;
    init_odom = false;
    return true;
}

void Rotation::update() {

    if ( init_odom )
    {

        // we receive a new /goal_to_reach
        if ( new_goal_to_reach )
            initRotation();

        //we are performing a rotation
        if ( cond_rotation )
        {
            computeRotation();
            moveRobot();
        }                

    }

}

void Rotation::initRotation()
{
    new_goal_to_reach = false;
    ROS_INFO("processing the /goal_to_reach received at (%f, %f)", goal_to_reach.x, goal_to_reach.y);

    // we have a rotation and a translation to perform
    // we compute the /translation_to_do
    translation_to_do = sqrt( ( goal_to_reach.x * goal_to_reach.x ) + ( goal_to_reach.y * goal_to_reach.y ) );

    if ( translation_to_do )
    {

        //we compute the /rotation_to_do
        rotation_to_do = acos( goal_to_reach.x / translation_to_do );

        if ( goal_to_reach.y < 0 )
            rotation_to_do *=-1;

        //we initialize the pid for the control of rotation
        initial_orientation = current_orientation;
        error_integral_rotation = 0;
        error_previous_rotation = 0;

        ROS_INFO("rotation_to_do: %f, translation_to_do: %f", rotation_to_do*180/M_PI, translation_to_do);
        cond_rotation = true;

    }
    else
        ROS_WARN("translation_to_do is equal to 0");

}

void Rotation::computeRotation()
{
    ROS_INFO("current_orientation: %f, initial_orientation: %f", current_orientation*180/M_PI, initial_orientation*180/M_PI);
    rotation_done = current_orientation - initial_orientation;

    //do not forget that rotation_done must always be between -M_PI and +M_PI

    error_rotation = current_orientation - rotation_done*M_PI/180;
    ROS_INFO("rotation_to_do: %f, rotation_done: %f, error_rotation: %f", rotation_to_do*180/M_PI, rotation_done*180/M_PI, error_rotation*180/M_PI);
    if( error_rotation < ROTATION_ERROR_THRESHOLD)
        cond_rotation = false; /*cond_rotation is used to control if we stop or not the pid*/

    rotation_speed = 0;

    if ( cond_rotation )
    {
        //Implementation of a PID controller for rotation_to_do;

        float error_derivation_rotation = error_rotation - error_previous_rotation; // divided by time-step
        error_previous_rotation = error_rotation;
        ROS_INFO("error_derivation_rotation: %f", error_derivation_rotation);

        error_integral_rotation += error_rotation;
        ROS_INFO("error_integral_rotation: %f", error_integral_rotation);

        //control of rotation with a PID controller
        rotation_speed = KP_R*error_rotation + KI_R *error_integral_rotation + KD_R * error_derivation_rotation;
        ROS_INFO("rotation_speed: %f", rotation_speed*180/M_PI);
    }
    else
        ROS_WARN("pid for rotation will stop");
}

void Rotation::moveRobot()
{

    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = rotation_speed;

    pub_cmd_vel.publish(twist);

}

void Rotation::goalToReachCallback(const geometry_msgs::Point::ConstPtr& g) {
// process the goal received from moving_persons detector
    new_goal_to_reach = true;
    goal_to_reach = *g;
}

}