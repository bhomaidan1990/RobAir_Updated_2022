#include <tutorial_ros/utility.h>
#include <tutorial_ros/robot_moving.h>

int nb_static = 5;

namespace robair
{
    /***********************************************************************************************************************
     * Class definitions: RobotMoving
     */

    /***********************************************************
     * Primary methods
     */
    RobotMoving::RobotMoving(ros::NodeHandle &nh) : nh_(nh)
    {
        init(nh_);

        ros::Rate r(20); // this node is updated at 20hz

        while (ros::ok())
        {
            ros::spinOnce();
            update();
            r.sleep();
        }
    }

    bool RobotMoving::init(ros::NodeHandle &nh)
    {
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

    void RobotMoving::update()
    {

        if (init_odom)
        { // we wait for new data of odometry
            init_odom = false;
            if ((not_moving_position.x == position.x) && (not_moving_position.y == position.y) && (not_moving_orientation == orientation))
            {
                count++;
                if ((count == nb_static) && (moving))
                {
                    ROS_INFO("robot is not moving");
                    moving = false;
                }
            }
            else
            {
                not_moving_position.x = position.x;
                not_moving_position.y = position.y;
                not_moving_orientation = orientation;
                count = 0;
                if (!moving)
                {
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