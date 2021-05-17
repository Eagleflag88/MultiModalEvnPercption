//
// Created by eagleflag on 2021/3/21.
//

#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>


#include "lidar_frontend_node.h"

// Declaration of Publishers

static ros::Publisher lidar_scan_pub;
static ros::Publisher chatter_pub;

// Declaration of Subscribers
static ros::Subscriber lidar_scan_sub;

pcl::PointCloud<pcl::PointXYZ>::Ptr pre_scan;

bool first_frame = false;

static void PointCloud_Callback(const sensor_msgs::PointCloud2 curr_scan_msg)
{
    ROS_INFO("PointCloudCallback Started");
    //Get current scan
    pcl::PointCloud<pcl::PointXYZ>::Ptr curr_scan(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(curr_scan_msg, *curr_scan); //Convert point cloud msg to point cloud pointer for incoming laser scan
    std::cout << "original number of point cloud is " << curr_scan->width << std::endl;

    if (!first_frame)
        // If the gnss_odom is not yet already initialized
    {
        std::cout << "PointCloud_Callback: gnss_odom is not yet initialized!" << std::endl;
        return;
    }

    //Get transform between two frames
    Eigen::Isometry3d pose_diff = Eigen::Isometry3d::Identity();
    register_pointcloud(curr_scan, pre_scan, pose_diff);

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "lidar_frontend_node");
    ros::NodeHandle nh;
    // Register the Subscriber
    // todo:Add a parameter loading class
    // %Tag(SUBSCRIBER)%
    lidar_scan_sub = nh.subscribe("/kitti/velo/pointcloud", 10, PointCloud_Callback);

    // %EndTag(SUBSCRIBER)%

    // %Tag(PUBLISHER)%
    chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
    lidar_scan_pub = nh.advertise<sensor_msgs::PointCloud2>("/lidar_scan", 10);

    int count = 0;

    std_msgs::String msg;
    while(ros::ok())
    {
        std::stringstream status_msg;
        status_msg << "lidar_frontend_node working fine " << count;
        msg.data = status_msg.str();
        ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(msg);
        ros::spin();
        count++;
    }

    return 0;
}