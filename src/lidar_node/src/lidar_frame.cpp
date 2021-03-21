//
// Created by eagleflag on 2021/3/21.
//

#include <memory>
#include <vector>
#include <deque>
#include <algorithm>
#include <chrono>
#include <string>
#include <sstream>
#include <iostream>
#include <cmath>
#include <fstream>

#include <pcl/filters/statistical_outlier_removal.h>
#include "lidar_frame.h"

lidar_frame::lidar_frame()
{
    std::cout << "lidar_frame without id " << "is constructed" << std::endl;
}

lidar_frame::lidar_frame(
        unsigned long id,
        double time_stamp,
        Eigen::Isometry3d T_cw,
        pcl::PointCloud<pcl::PointXYZ>::Ptr scan
):
        id_(id),
        time_stamp_(time_stamp),
        T_cw_(T_cw),
        scan_(scan)// Initialization of the constructor
{
    std::cout << "frame with id " << id_ << " is constructed" << std::endl;
}

lidar_frame::~lidar_frame()
{
    std::cout << "lidar_frame with id " << id_ << " is destructed" << std::endl;
}

lidar_frame::ptr lidar_frame::create_frame()
{
    static unsigned long lidar_frame_id = 0;
    return lidar_frame::ptr( new lidar_frame(lidar_frame_id++) ); // ptr here is the shared pointer defined with typedef
}

//todo: undistortion
//todo: z-axis error
void lidar_frame::filter()
{
    // Downsampling
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize(0.4, 0.4, 0.4);
    approximate_voxel_filter.setInputCloud(scan_);
    auto t1 = ros::WallTime::now();
    approximate_voxel_filter.filter(*scan_);
    auto t2 = ros::WallTime::now();
    std::cout << "Duration for Downsampling: " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;
    std::cout << "Point Cloud after Downsampling " << scan_->width << std::endl;

    // Outlier Detection and Removal
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (scan_);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    t1 = ros::WallTime::now();
    sor.filter (*scan_);
    t2 = ros::WallTime::now();
    std::cout << "Duration for Outlier Removal: " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;
    std::cout << "Point Cloud after Outlier Removal contains " << scan_->width << std::endl;
}

void lidar_frame::pub_odom(ros::Publisher odom_pub)
{
    std::cout << "Publishing the odom "<< std::endl;
    // update the position relative to first frame
    double pose_x = T_cw_.matrix()(0, 3);
    double pose_y = T_cw_.matrix()(1, 3);
    double pose_z = T_cw_.matrix()(2, 3);

    // update the rotation matrix relative to first frame
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix(0, 0) = T_cw_.matrix()(0, 0);
    rotation_matrix(0, 1) = T_cw_.matrix()(0, 1);
    rotation_matrix(0, 2) = T_cw_.matrix()(0, 2);
    rotation_matrix(1, 0) = T_cw_.matrix()(1, 0);
    rotation_matrix(1, 1) = T_cw_.matrix()(1, 1);
    rotation_matrix(1, 2) = T_cw_.matrix()(1, 2);
    rotation_matrix(2, 0) = T_cw_.matrix()(2, 0);
    rotation_matrix(2, 1) = T_cw_.matrix()(2, 1);
    rotation_matrix(2, 2) = T_cw_.matrix()(2, 2);

    // Update the Quaternion
    Eigen::Quaterniond q = Eigen::Quaterniond(rotation_matrix);
    double pose_qx = q.coeffs()(0);
    double pose_qy = q.coeffs()(1);
    double pose_qz = q.coeffs()(2);
    double pose_qw = q.coeffs()(3);


    // publish the lidar odometry /////////////////////////////////////////////////////////////////////////////////////////
    nav_msgs::Odometry curr_odom;
//    curr_odom.header.stamp = lcam_img_msg_ptr->header.stamp;
    curr_odom.header.frame_id = "imu_link";
    curr_odom.child_frame_id = "lidar_odom";

    curr_odom.pose.pose.position.x = pose_x;
    curr_odom.pose.pose.position.y = pose_y;
    curr_odom.pose.pose.position.z = pose_z;

    curr_odom.pose.pose.orientation.x = pose_qx;
    curr_odom.pose.pose.orientation.y = pose_qy;
    curr_odom.pose.pose.orientation.z = pose_qz;
    curr_odom.pose.pose.orientation.w = pose_qw;

    odom_pub.publish(curr_odom);

    std::cout << "Finishing Publishing "<< std::endl;
}