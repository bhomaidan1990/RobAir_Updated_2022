#include <tutorial_ros/utility.h>
#include <follow_me/obstacle_detection.h>

namespace robair{
/***********************************************************************************************************************
 * Class definitions: ObstacleDetection
 */

/***********************************************************
 * Primary methods
 */
ObstacleDetection::ObstacleDetection(ros::NodeHandle& nh):
  nh_(nh) 
  {
    init(nh_);

    //INFINTE LOOP TO COLLECT LASER DATA AND PROCESS THEM
    ros::Rate r(10);// this node will run at 10hz
    while (ros::ok()) {
        ros::spinOnce();//each callback is called once to collect new data: laser + robot_moving
        update();//processing of data
        r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }

}

bool ObstacleDetection::init(ros::NodeHandle& nh){
    // To Do Some Checks and Initializations.
    ROS_INFO("Obstacle Detection Node Initialization...");

    // name_space_ = "/follow_me";
    sub_scan_ = nh.subscribe("/scan", 1, &scanCallback);

    new_laser = false;

    // communication with translation_action
    pub_closest_obstacle = nh.advertise<geometry_msgs::Point>("closest_obstacle", 1);
    pub_closest_obstacle_marker = nh.advertise<visualization_msgs::Marker>("closest_obstacle_marker", 1); // Preparing a topic to publish our results. This will be used by the visualization tool rviz

    return true;
}

void ObstacleDetection::update(){

    float robair_size = 0.25;

    if ( new_laser )
    {
        closest_obstacle.x = range_max;
        closest_obstacle.y = range_max;

        float beam_angle = angle_min;
        for ( int loop=0; loop < nb_beams; loop++, beam_angle += angle_inc ) {
            //ROS_INFO("hit[%i]: (%f, %f) -> (%f, %f)", loop, range[loop], beam_angle*180/M_PI, current_scan[loop].x, current_scan[loop].y);
            if ( ( fabs(current_scan[loop].y) < robair_size ) && ( fabs(closest_obstacle.x) > fabs(current_scan[loop].x) ) && ( current_scan[loop].x > 0 ) )
                closest_obstacle = current_scan[loop];
        }

        pub_closest_obstacle.publish(closest_obstacle);

        int nb_pts=0;
        // closest obstacle is red
        display[nb_pts] = closest_obstacle;

        colors[nb_pts].r = 1;
        colors[nb_pts].g = 0;
        colors[nb_pts].b = 0;
        colors[nb_pts].a = 1.0;
        nb_pts++;
        if(nb_pts>0)
            populateMarkerTopic(pub_closest_obstacle_marker, nb_pts, display, colors);

        if ( distancePoints(closest_obstacle, previous_closest_obstacle) > 0.05 ) 
        {
            ROS_INFO("closest obstacle: (%f; %f)", closest_obstacle.x, closest_obstacle.y);

            previous_closest_obstacle.x = closest_obstacle.x;
            previous_closest_obstacle.y = closest_obstacle.y;
        }
    }
}

}