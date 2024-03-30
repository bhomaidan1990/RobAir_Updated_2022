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

    LaserTextDisplay::LaserTextDisplay(ros::NodeHandle &nh) : nh_(nh)
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

    bool LaserTextDisplay::init(ros::NodeHandle &nh)
    {
        // To Do Some Checks and Initializations.
        ROS_INFO("Laser Text Display Node Initialization...");
        // name_space_ = "/tutorial_ros";

        sub_scan_ = nh.subscribe("/scan", 1, &scanCallback);

        new_laser = false;

        return true;
    }

    void LaserTextDisplay::update()
    {
        // we wait for new data of the laser
        if (new_laser)
        {
            ROS_INFO("New data of laser received");

            for (int loop = 0; loop < nb_beams; loop++)
                ROS_INFO("r[%i] = %f, theta[%i] (in degrees) = %f, x[%i] = %f, y[%i] = %f", loop,
                         r[loop], loop, theta[loop] * 180 / M_PI, loop, current_scan[loop].x, loop, current_scan[loop].y);

            new_laser = false;
        }
    }

}