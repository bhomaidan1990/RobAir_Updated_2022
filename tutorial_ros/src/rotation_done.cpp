// Security Distance
#define SECURITY_DISTANCE 0.5

#include <tutorial_ros/utility.h>
#include <tutorial_ros/rotation_done.h>

namespace robair
{
    /***********************************************************************************************************************
     * Class definitions: RotationDone
     */

    /***********************************************************
     * Primary methods
     */
    RotationDone::RotationDone(ros::NodeHandle &nh) : nh_(nh)
    {
        init(nh_);

        // INFINITE LOOP TO COLLECT LASER DATA AND PROCESS THEM
        ros::Rate r(10); // this node will run at 10hz
        while (ros::ok())
        {
            ros::spinOnce(); // each callback is called once to collect new data: laser + robot_moving
            update();        // processing of data
            r.sleep();       // we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
        }
    }

    bool RotationDone::init(ros::NodeHandle &nh)
    {
        // To Do Some Checks and Initializations.
        ROS_INFO("Rotation Done Node Initialization...");

        // name_space_ = "/tutorial_ros";

        // communication with odometry
        sub_odometry_ = nh.subscribe("/odom", 1, &odomCallback);

        init_odom = false;

        first = true;

        rotation_done = 0;

        return true;
    }

    void RotationDone::update()
    {

        if (init_odom)
        {

            if (first)
                initial_orientation = orientation;
            first = false;

            rotation_done = orientation - initial_orientation;

            // do not forget that rotation_done must always be between -M_PI and +M_PI
            if (rotation_done > M_PI)
            {
                ROS_WARN("rotation_done > 180 degrees: %f degrees -> %f degrees", rotation_done * 180 / M_PI, (rotation_done - 2 * M_PI) * 180 / M_PI);
                rotation_done -= 2 * M_PI;
            }
            else if (rotation_done < -M_PI)
            {
                ROS_WARN("rotation_done < -180 degrees: %f degrees -> %f degrees", rotation_done * 180 / M_PI, (rotation_done + 2 * M_PI) * 180 / M_PI);
                rotation_done += 2 * M_PI;
            }

            ROS_INFO("current_orientation: %f, initial_orientation: %f, rotation_done: %f", orientation * 180 / M_PI, initial_orientation * 180 / M_PI, rotation_done * 180 / M_PI);
        }
    }

}
