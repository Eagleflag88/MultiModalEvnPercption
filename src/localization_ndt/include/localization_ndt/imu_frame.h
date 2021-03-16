//
// Created by eagleflag on 2020/9/16.
//

#ifndef CATKIN_WS_NDT_IMU_FRAME_H
#define CATKIN_WS_NDT_IMU_FRAME_H

#include <Eigen/Geometry>
#include <Eigen/Core>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "ros/ros.h"

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include "data_type.h"

class imu_frame
{
public:
    typedef std::shared_ptr<imu_frame> ptr;
    unsigned long id_;
    double time_stamp_;
    Eigen::Isometry3d T_cw_;
    Eigen::Matrix3d rotation_matrix_;
    Eigen::Vector3d vel_global_;
    Eigen::Vector3d accel_global_;
    imu_data frame_; // 初始化imu_data

public:
    imu_frame();
    imu_frame(
            unsigned long id,
            double time_stamp = 0.0,
            Eigen::Isometry3d T_cw = Eigen::Isometry3d::Identity(),
            Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Zero(),
            Eigen::Vector3d vel_global = Eigen::Vector3d::Zero(),
            Eigen::Vector3d accel_global = Eigen::Vector3d::Zero(),
            imu_data frame = {}
            );
    ~imu_frame();
    ptr create_frame();
    void update_frame(const sensor_msgs::ImuConstPtr& imu_msg_ptr);
    void pub_odom(ros::Publisher odom_pub);

};

#endif //CATKIN_WS_NDT_IMU_FRAME_H
