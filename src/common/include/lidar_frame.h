//
// Created by eagleflag on 2021/3/21.
//

#ifndef MULTIMODALEVNPERCPTION_LIDAR_FRAME_H
#define MULTIMODALEVNPERCPTION_LIDAR_FRAME_H

#include "ros/ros.h"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>


typedef pcl::PointXYZI PointType;

class lidar_frame
{
public:
    typedef std::shared_ptr<lidar_frame> ptr;
    unsigned long id_;
    double time_stamp_;
    Eigen::Isometry3d T_cw_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_;
    float cloudCurvature[400000];
    int cloudSortInd[400000];
    int cloudNeighborPicked[400000];
    int cloudLabel[400000];

public:
    lidar_frame();
    lidar_frame(
            unsigned long id,
            double time_stamp = 0.0,
            Eigen::Isometry3d T_cw = Eigen::Isometry3d::Identity(),
            pcl::PointCloud<pcl::PointXYZ>::Ptr scan = nullptr
    );
    ~lidar_frame();
    ptr create_frame();
    void filter();
    void loam_feature();
    void pub_odom(ros::Publisher odom_pub);

private:
    const double MINIMUM_RANGE = 0.1;
    int N_SCANS = 16;
    const double scanPeriod = 0.1;

};

#endif //MULTIMODALEVNPERCPTION_LIDAR_FRAME_H
