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


#include "lidar_frame.h"

// Declaration of Publishers

static ros::Publisher lidar_scan_pub;
static ros::Publisher chatter_pub;

// Declaration of Subscribers
static ros::Subscriber lidar_scan_sub;

//todo: 查看所有容器的使用，复制还是引用
//todo: lock for data race

static void PointCloud_Callback(const sensor_msgs::PointCloud2 curr_scan_msg)
{
    ROS_INFO("PointCloudCallback Started");
    //Get current scan
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(curr_scan_msg, *scan_cloud_ptr); //Convert point cloud msg to point cloud pointer for incoming laser scan
    std::cout << "original number of point cloud is " << scan_cloud_ptr->width << std::endl;

    lidar_frame::ptr lidar_frame_ptr = lidar_frame_ptr->create_frame();
    lidar_frame_ptr->scan_ = scan_cloud_ptr;
    lidar_frame_ptr->filter();
    lidar_frame_ptr->loam_feature();

    std::cout << "id of the cloud point frame ptr is " << lidar_frame_ptr->id_ << std::endl;

    return;

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "lidar_node");
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
        status_msg << "lidar_node working fine " << count;
        msg.data = status_msg.str();
        ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(msg);
        ros::spin();
        count++;
    }

    return 0;
}