//
// Created by eagleflag on 2020/9/17.
//

#ifndef CATKIN_WS_NDT_GNSS_FRAME_H
#define CATKIN_WS_NDT_GNSS_FRAME_H

#include <Eigen/Geometry>
#include <Eigen/Core>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "ros/ros.h"

#include <nav_msgs/Odometry.h>
#include "data_type.h"

#include "Geocentric/LocalCartesian.hpp"

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>

class map; // forward declaration：因为需要在不互相include头文件的情况下使用map中的数据类型
class gnss_frame
{
public:
    typedef std::shared_ptr<gnss_frame> ptr;
    unsigned long id_;
    double time_stamp_;
    Eigen::Isometry3d T_cw_;
    gnss_data frame_; // 初始化gnss_data

    static GeographicLib::LocalCartesian geo_converter_;


public:
    gnss_frame();
    gnss_frame(
            unsigned long id,
            double time_stamp = 0.0,
            Eigen::Isometry3d T_cw = Eigen::Isometry3d::Identity(),
            gnss_data frame = {}
    );
    ~gnss_frame();
    ptr create_frame();
    void set_origin(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr, const std::shared_ptr<map>& map_ptr);
    void set_frame(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr, const std::shared_ptr<map>& map_ptr);
    void pub_odom(ros::Publisher odom_pub);


};


#endif //CATKIN_WS_NDT_GNSS_FRAME_H
