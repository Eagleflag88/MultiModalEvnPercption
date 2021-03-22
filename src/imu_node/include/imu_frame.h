//
// Created by eagleflag on 2021/3/22.
//

#ifndef MULTIMODALEVNPERCPTION_IMU_FRAME_H
#define MULTIMODALEVNPERCPTION_IMU_FRAME_H

#include <Eigen/Geometry>
#include <Eigen/Core>

#include "ros/ros.h"

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include "imu_data.h"

class imu_frame
{
public:
    typedef std::shared_ptr<imu_frame> ptr;
    unsigned long id_;
    double time_stamp_;
    imu_data frame_; // 初始化imu_data

public:
    imu_frame();
    imu_frame(
            unsigned long id,
            double time_stamp = 0.0,
            imu_data frame = {}
    );
    ~imu_frame();
    ptr create_frame();
    void update_frame(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

};

#endif //MULTIMODALEVNPERCPTION_IMU_FRAME_H
