/**********************************************************
 * Copyright (c) 2024, GIPSA / COPERNIC Team / O. Aycard
 * All rights reserved.
 **********************************************************/

#pragma once

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

namespace robair
{

class Localization{
public:
    /**
     * \brief Default Class Constructor.
     */
    Localization(ros::NodeHandle& nh);

    /**
     * \brief Initialization.
     */
    bool init(ros::NodeHandle& nh);
    
    /**
     * \brief Laser Data Processing.
     */
    void update();

    /**
     * \brief TODO.
     */
    void initializeLocalization();

    /**
     * \brief TODO.
     */
    void predictPosition();

    /**
     * \brief TODO.
     */
    void estimatePosition();

    /**
     * \brief TODO.
     * \return Something.
     */
    int sensorModel(float x, float y, float o);

    /**
     * \brief TODO.
     * \return Something.
     */
    int cellValue(float x, float y);

    /**
     * \brief TODO.
     */
    void positionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& p);
        
private:
    /**
     * \brief Node Handler.
     */
    ros::NodeHandle nh_;

    /**
     * \brief TODO.
     */
    ros::Publisher pub_localization_marker;

    /**
     * \brief TODO.
     */
    ros::Subscriber sub_position;

    /**
     * \brief TODO.
     */
    //to store the map
    nav_msgs::GetMap::Response resp;

    /**
     * \brief TODO.
     */
    geometry_msgs::Point min, max;

    /**
     * \brief TODO.
     */
    float cell_size;

    /**
     * \brief TODO.
     */
    int width_max;

    /**
     * \brief TODO.
     */
    int height_max;

    /**
     * \brief TODO.
     */
    // GRAPHICAL DISPLAY
    int nb_pts;

    /**
     * \brief TODO.
     */
    geometry_msgs::Point odom_current;

    /**
     * \brief TODO.
     */
    float odom_current_orientation;

    /**
     * \brief TODO.
     */
    geometry_msgs::Point odom_last;

    /**
     * \brief TODO.
     */
    float odom_last_orientation;

    /**
     * \brief TODO.
     */
    //to store the initial_position of the mobile robot
    bool init_position;

    /**
     * \brief TODO.
     */
    geometry_msgs::Point initial_position;

    /**
     * \brief TODO.
     */
    float initial_orientation;

    /**
     * \brief TODO.
     */
    //to store the predicted and estimated position of the mobile robot
    bool localization_initialized;

    /**
     * \brief TODO.
     */
    geometry_msgs::Point predicted_position;

    /**
     * \brief TODO.
     */
    float predicted_orientation;

    /**
     * \brief TODO.
     */
    geometry_msgs::Point estimated_position;

    /**
     * \brief TODO.
     */
    float estimated_orientation;

    /**
     * \brief TODO.
     */
    float distance_traveled;

    /**
     * \brief TODO.
     */
    float previous_distance_traveled;

    /**
     * \brief TODO.
     */
    float angle_traveled;

    /**
     * \brief TODO.
     */
    float previous_angle_traveled;
};
}
