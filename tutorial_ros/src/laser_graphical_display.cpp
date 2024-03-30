#include <tutorial_ros/utility.h>
#include <tutorial_ros/laser_graphical_display.h>

namespace robair
{
/***********************************************************************************************************************
 * Class definitions: LaserGraphicalDisplay
 */

/***********************************************************
 * Primary methods
 */
LaserGraphicalDisplay::LaserGraphicalDisplay(ros::NodeHandle& nh):
  nh_(nh) 
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

bool LaserGraphicalDisplay::init(ros::NodeHandle& nh){
    // To Do Some Checks and Initializations.
    ROS_INFO("Laser Laser Graphical Display Node Initialization...");
    
    // name_space_ = "/tutorial_ros";

      sub_scan_ = nh.subscribe("/scan", 1, &scanCallback);

      // Preparing a topic to publish our results. This will be used by the visualization tool rviz
      pub_laser_graphical_display_marker_ = nh.advertise<visualization_msgs::Marker>("laser_graphical_display_marker", 1);

      new_laser = false;
      
    return true;
}

void LaserGraphicalDisplay::update() {

    // we wait for new data of the laser
    if ( new_laser )
    {

        new_laser = false;
        ROS_INFO("New data of laser received");

        int nb_pts = 0;
        for ( int loop=0 ; loop < nb_beams; loop++ )
        {
            ROS_INFO("r[%i] = %f, theta[%i] (in degrees) = %f, x[%i] = %f, y[%i] = %f", loop, r[loop], loop, theta[loop]*180/M_PI, loop, current_scan[loop].x, loop, current_scan[loop].y);

            display[nb_pts] = current_scan[loop];

            colors[nb_pts].r = 0;
            colors[nb_pts].g = 0;
            colors[nb_pts].b = 1;
            colors[nb_pts].a = 1.0;

            nb_pts++;
        }
        if(nb_pts>0)
            populateMarkerTopic(pub_laser_graphical_display_marker_, nb_pts, display, colors);
    }
}

}