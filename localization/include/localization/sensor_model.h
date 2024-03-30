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

class SensorModel{
public:
    /**
     * \brief Default Class Constructor.
     */
    SensorModel(ros::NodeHandle& nh);

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
    ros::Subscriber sub_position;

    /**
     * \brief TODO.
     */
    ros::Publisher pub_sensor_model_marker;

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
    //to store the initial_position of the mobile robot
    bool init_position;
    
    /**
     * \brief TODO.
     */
    bool new_position;

    /**
     * \brief TODO.
     */
    geometry_msgs::Point initial_position;

    /**
     * \brief TODO.
     */
    float initial_orientation;
};
}
