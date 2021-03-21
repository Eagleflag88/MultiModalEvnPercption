//
// Created by eagleflag on 2021/3/20.
//

#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include <deque>
#include <vector>
#include <algorithm>
#include <chrono>

#include "gnss_data.h"
#include "gnss_frame.h"
//#include "Geocentric/LocalCartesian.hpp"

// Declaration of Publishers

static ros::Publisher gnss_odom_pub;
static ros::Publisher chatter_pub;

// Declaration of Subscribers
static ros::Subscriber gnss_sub;

GeographicLib::LocalCartesian gnss_frame::geo_converter_ = GeographicLib::LocalCartesian(0.0, 0.0, 0.0);

//todo: 查看所有容器的使用，复制还是引用
//todo: lock for data race

bool gnss_odom_initialized = false;

static void GNSS_Callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr)
{
    if (!gnss_odom_initialized)
        // if the odom is not yet initialized with GNSS Data
    {
        std::cout << "initialization of gnss odom" << std::endl;

        gnss_frame::ptr gnss_frame_ptr = gnss_frame_ptr->create_frame();
        gnss_frame_ptr->set_origin(nav_sat_fix_ptr);
        gnss_frame_ptr->pub_odom(gnss_odom_pub);

        gnss_odom_initialized = true;
    }
    else
        // If the odom is initialized with GNSS Data
    {
        std::cout << "odom already initialized" << std::endl;
        //update the position with GNSS Data

        gnss_frame::ptr gnss_frame_ptr = gnss_frame_ptr->create_frame();
        gnss_frame_ptr->set_frame(nav_sat_fix_ptr);
        // publisher the gnss_odom msg
        gnss_frame_ptr->pub_odom(gnss_odom_pub);
    }

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "gnss_node");
    ros::NodeHandle nh;
    // Register the Subscriber
    // todo:Add a parameter loading class
    gnss_sub = nh.subscribe("/kitti/oxts/gps/fix", 100, GNSS_Callback);

    // Register the Publisher
    gnss_odom_pub = nh.advertise<nav_msgs::Odometry>("/gnss_odom", 50);

    int count = 0;

    std_msgs::String msg;
    while(ros::ok())
    {
        std::stringstream status_msg;
        status_msg << "gnss_node working fine " << count;
        msg.data = status_msg.str();
        ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(msg);
        ros::spin();
        count++;
    }

    return 0;
}



