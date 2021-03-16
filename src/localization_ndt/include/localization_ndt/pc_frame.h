//
// Created by eagleflag on 2020/9/9.
//

#ifndef CATKIN_WS_NDT_PC_FRAME_H
#define CATKIN_WS_NDT_PC_FRAME_H

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Image.h>

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

#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>



class pc_frame
{
public:
    typedef std::shared_ptr<pc_frame> ptr;
    unsigned long id_;
    double time_stamp_;
    Eigen::Isometry3d T_cw_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_;

public:
    pc_frame();
    pc_frame(
            unsigned long id,
            double time_stamp = 0.0,
            Eigen::Isometry3d T_cw = Eigen::Isometry3d::Identity(),
            pcl::PointCloud<pcl::PointXYZ>::Ptr scan = nullptr
            );
    ~pc_frame();
    ptr create_frame();
    void filter();
    void pub_odom(ros::Publisher odom_pub);

};

#endif //CATKIN_WS_NDT_PC_FRAME_H
