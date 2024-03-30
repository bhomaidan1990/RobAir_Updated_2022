/**********************************************************
 * Copyright (c) 2024, GIPSA / COPERNIC Team / O. Aycard
 * All rights reserved.
 **********************************************************/

#pragma once

#include "ros/ros.h"
#include "geometry_msgs/Point.h"

namespace robair
{

class DATMO{
public:
    /**
     * \brief Default Class Constructor.
     */
    DATMO(ros::NodeHandle& nh);

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
    void storeBackground();
    
    /**
     * \brief TODO.
     */
    void resetMotion();

    /**
     * \brief TODO.
     */
    void detectMotion();

    /**
     * \brief TODO.
     */
    void performClustering();

    /**
     * \brief TODO.
     */
    void detectLegs();

    /**
     * \brief TODO.
     */
    void detectPersons();

    /**
     * \brief TODO.
     */
    void detectMovingPerson();

    /**
     * \brief TODO.
     */
    void trackMovingPerson();

private:
    /**
     * \brief Node Handler.
     */
    ros::NodeHandle nh_;
    
    /**
     * \brief TODO.
     */
    ros::Publisher pub_datmo;

    /**
     * \brief TODO.
     */
    ros::Publisher pub_datmo_marker;

    /**
     * \brief TODO.
     */
    //to perform detection of motion
    bool stored_background;
    
    /**
     * \brief TODO.
     */
    float background[1000];// to store the range of each hit
    
    /**
     * \brief TODO.
     */
    bool dynamic[1000];//to know if the corresponding hit is dynamic or not

    /**
     * \brief TODO.
     */
    //to perform clustering
    int nb_cluster;// number of cluster
    
    /**
     * \brief TODO.
     */
    int cluster[1000]; //to store for each hit, the cluster it belongs to
    
    /**
     * \brief TODO.
     */
    float cluster_size[1000];//to store the size of each cluster
    
    /**
     * \brief TODO.
     */
    geometry_msgs::Point cluster_middle[1000];// to store the middle of each cluster
    
    /**
     * \brief TODO.
     */
    int cluster_dynamic[1000];//to store the percentage of the cluster that is dynamic
    
    /**
     * \brief TODO.
     */
    int cluster_start[1000], cluster_end[1000];

    /**
     * \brief TODO.
     */
    //to perform detection of legs and to store them
    int nb_legs_detected;
    
    /**
     * \brief TODO.
     */
    geometry_msgs::Point leg_detected[1000];//to store the coordinates a leg
    
    /**
     * \brief TODO.
     */
    int leg_cluster[1000];//to store the cluster corresponding to a leg
    
    /**
     * \brief TODO.
     */
    bool leg_dynamic[1000];//to know if a leg is dynamic or not

    /**
     * \brief TODO.
     */
    //to perform detection of a moving person and store it
    int nb_persons_detected;
    
    /**
     * \brief TODO.
     */
    geometry_msgs::Point person_detected[1000];//to store the coordinates of a person
    
    /**
     * \brief TODO.
     */
    bool person_dynamic[1000];//to know if a person is dynamic or not
    
    /**
     * \brief TODO.
     */
    geometry_msgs::Point moving_person_tracked;//to store the coordinates of the moving person that we are tracking

    /**
     * \brief TODO.
     */
    //to perform tracking of the moving person
    bool tracking_mode;//to know if we are tracking a moving person or not
    
    /**
     * \brief TODO.
     */
    float uncertainty;

    /**
     * \brief TODO.
     */
    int frequency;

    /**
     * \brief TODO.
     */
    int nb_pts;
};
}
