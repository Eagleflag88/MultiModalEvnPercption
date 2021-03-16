//
// Created by eagleflag on 2020/7/8.
//

#ifndef CATKIN_WS_NDT_POINT_CLOUD_DATA_H
#define CATKIN_WS_NDT_POINT_CLOUD_DATA_H

#include "ros/ros.h"
#include <pcl/point_types.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

struct point_cloud
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr;
    double time = 0.0;
};

struct lcam_data
{
    cv_bridge::CvImagePtr lcam_data_ptr;
//    double time = 0.0;
};

struct gnss_data
{
    double time = 0.0;
    double longitude = 0.0;
    double latitude = 0.0;
    double altitude = 0.0;
    double local_E = 0.0;
    double local_N = 0.0;
    double local_U = 0.0;
    int status = 0;
    int service = 0;

};

struct imu_data
{
    double time = 0.0;
    double linear_acceleration_x = 0.0;
    double linear_acceleration_y = 0.0;
    double linear_acceleration_z = 0.0;

    double angular_velocity_x = 0.0;
    double angular_velocity_y = 0.0;
    double angular_velocity_z = 0.0;

    double orientation_x = 0.0;
    double orientation_y = 0.0;
    double orientation_z = 0.0;
    double orientation_w = 1.0;
};


#endif //CATKIN_WS_NDT_POINT_CLOUD_DATA_H


