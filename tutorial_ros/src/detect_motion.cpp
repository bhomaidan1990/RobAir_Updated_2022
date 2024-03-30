// threshold for detection of motion
#define DETECTION_THRESHOLD 0.2

#include <tutorial_ros/utility.h>
#include <tutorial_ros/detect_motion.h>

namespace robair
{
    /***********************************************************************************************************************
     * Class definitions: DetectMotion
     */

    /***********************************************************
     * Primary methods
     */
    DetectMotion::DetectMotion(ros::NodeHandle &nh) : nh_(nh)
    {
        init(nh_);

        // INFINTE LOOP TO COLLECT LASER DATA AND PROCESS THEM
        ros::Rate r(10); // this node will run at 10hz
        while (ros::ok())
        {
            ros::spinOnce(); // each callback is called once to collect new data: laser + robot_moving
            update();        // processing of data
            r.sleep();       // we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
        }
    }

    bool DetectMotion::init(ros::NodeHandle &nh)
    {
        // To Do Some Checks and Initializations.
        ROS_INFO("Detect Motion Node Initialization...");

        // name_space_ = "/tutorial_ros";

        sub_scan_ = nh.subscribe("/scan", 1, &scanCallback);

        // TODO: check topic namespace!
        sub_robot_moving_ = nh.subscribe("/tutorial_ros/robot_moving", 1, &robotMovingCallback);

        // Preparing a topic to publish our results. This will be used by the visualization tool rviz
        pub_detect_motion_marker_ = nh.advertise<visualization_msgs::Marker>("detect_motion_marker", 1);

        new_laser = false;

        init_robot = false;

        previous_robot_moving = true;

        stored_background = false;

        return true;
    }

    void DetectMotion::update()
    {

        // we wait for new data of the laser and of the robot_moving_node to perform laser processing
        if (new_laser && init_robot)
        {
            ROS_INFO("\n");
            ROS_INFO("New data of laser received");
            ROS_INFO("New data of robot_moving received");

            if (!current_robot_moving)
            {
                // if the robot is not moving then we can perform moving person detection
                if (stored_background)
                    motionDetection();
                // DO NOT FORGET to store the background but when ???
                if (previous_robot_moving)
                    storeBackground();
                ROS_INFO("robot is not moving");
            }
            else
            {
                // IMPOSSIBLE TO DETECT MOTIONS because the base is moving
                // what is the value of dynamic table for each hit of the laser ?
                ROS_INFO("robot is moving");
            }
            previous_robot_moving = current_robot_moving;
        }
        else if (!init_robot)
            ROS_WARN("waiting for robot_moving_node");
    }

    void DetectMotion::storeBackground()
    {
        // store for each hit of the laser its range r in the background table

        ROS_INFO("storing background");
        /*== TODO: Fill here ==*/
        /*for (int loop=0; loop<nb_beams; loop++)
            background[loop] = ...;*/
        for (int loop = 0; loop < nb_beams; loop++)
            background[loop] = r[loop];

        stored_background = true;

        ROS_INFO("background stored");
    }

    void DetectMotion::motionDetection()
    {

        ROS_INFO("detecting motion");
        int nb_dynamic_pts = 0;

        for (int loop = 0; loop < nb_beams; loop++)
        { // loop over all the hits
            /*== TODO: Fill here ==*/
            // if the difference between ( the background and the current value r ) is higher than "detection_threshold"
            // then
            if ((background[loop] - r[loop]) > DETECTION_THRESHOLD)
                dynamic[loop] = true; // the current hit is dynamic
            else
                dynamic[loop] = false; // else its static

            if (dynamic[loop])
            {

                ROS_INFO("hit[%i](%f, %f) is dynamic", loop, current_scan[loop].x, current_scan[loop].y);

                // display in blue of hits that are dynamic
                display[nb_dynamic_pts] = current_scan[loop];

                colors[nb_dynamic_pts].r = 1;
                colors[nb_dynamic_pts].g = 0;
                colors[nb_dynamic_pts].b = 1;
                colors[nb_dynamic_pts].a = 1.0;

                nb_dynamic_pts++;
            }
        }
        // graphical display of the results
        if (nb_dynamic_pts > 0)
        {
            populateMarkerTopic(pub_detect_motion_marker_, nb_dynamic_pts, display, colors);
            ROS_INFO("motion detected");
        }
    }

}