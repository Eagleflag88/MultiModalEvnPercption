//
// Created by eagleflag on 2021/5/17.
//

#ifndef MULTIMODALEVNPERCPTION_LIDAR_FRONTEND_NODE_H
#define MULTIMODALEVNPERCPTION_LIDAR_FRONTEND_NODE_H

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

#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>

#include <Eigen/Geometry>
#include <Eigen/Core>

#include <opencv2/core/core.hpp>
//#include <opencv2/core/eigen.hpp>

#include "lidar_frame.h"

const int NumThreads = 10;
const double epsilon = 0.1;
const int step = 1;
const double res = 10.0;
const int iter = 60;

void register_pointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr curr_scan,
              const pcl::PointCloud<pcl::PointXYZ>::Ptr pre_scan,
              Eigen::Isometry3d& pose_diff)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_scan_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    // Using Normal NDT
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setTransformationEpsilon(0.01);
    ndt.setStepSize(0.1);
    ndt.setResolution(1.0);
    ndt.setMaximumIterations(35);
    ndt.setInputCloud(curr_scan);
    ndt.setInputTarget(pre_scan);
//    ndt.setInputTarget(local_pc_map_);
    auto t1 = ros::WallTime::now();
    ndt.align(*aligned_scan_cloud_ptr);
    auto t2 = ros::WallTime::now();
    std::cout << "Duration : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;

    std::cout << "Normal Distributions Transform has converged after " << ndt.getFinalNumIteration() << " iters" << std::endl;
    std::cout << "The score is " << ndt.getFitnessScore() << "\n" << std::endl;
    Eigen::Matrix4f Transf = Eigen::Matrix4f::Identity();
    Transf = ndt.getFinalTransformation();
    std::cout << "Final Transformation Matrix of the Scan Registration is " << Transf << std::endl;

//    //Get transform between two frames Using ndt_omp ////////////////////////////////////////////////////////////////////
//    pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
//    ndt_omp->setInputSource(curr_scan);
//    //    ndt_omp->setInputTarget(local_pc_map_);
//    ndt_omp->setInputTarget(pre_scan);
//    ndt_omp->setTransformationEpsilon (epsilon);
//    ndt_omp->setResolution (res);
//    ndt_omp->setStepSize (step);
//    ndt_omp->setMaximumIterations(100);
//
//    ndt_omp->setNumThreads(NumThreads);
//    ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
//    // get the transform between two poses as guess
//    std::cout << "Preparation for Scan Registration Finished" << std::endl;
//    auto t1 = ros::WallTime::now();
//    ndt_omp->align(*aligned_scan_cloud_ptr);
//    auto t2 = ros::WallTime::now();
//    std::cout << "Duration : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;
//
//    std::cout << "Normal Distributions Transform has converged after " << ndt_omp->getFinalNumIteration() << " iters" << std::endl;
//    std::cout << "The score is " << ndt_omp->getFitnessScore() << std::endl;
//
//    Eigen::Matrix4f Transf = Eigen::Matrix4f::Identity();
//    Transf = ndt_omp->getFinalTransformation();

//    // Using GPU normal//////////////////////////////////////////////////////////////////////////////////////////
//    gpu::GNormalDistributionsTransform g_ndt;
//    g_ndt.setTransformationEpsilon (epsilon);
//    g_ndt.setStepSize (step);
//    g_ndt.setResolution (res);
//    // Setting max number of registration iterations.
//    g_ndt.setMaximumIterations (35);
//
//    g_ndt.setInputSource (curr_scan);
//    g_ndt.setInputTarget (pre_scan);
//    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
//    auto t1 = ros::WallTime::now();
//    g_ndt.align(init_guess);
//    auto t2 = ros::WallTime::now();
//    std::cout << "Duration : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;
//
//    Eigen::Matrix4f Transf = Eigen::Matrix4f::Identity();
//    Transf = g_ndt.getFinalTransformation();
//    bool converged = g_ndt.hasConverged();
//    double fitness_score = g_ndt.getFitnessScore();

    // Output ////////////////////////////////////////////////////////////

    std::cout << "Final Transformation Matrix of the Scan Registation is " << std::endl << Transf << std::endl;
    cv::Mat t = (cv::Mat_<double>(3,1) << Transf(0, 3), Transf(1, 3), Transf(2, 3));
    std::cout << "Mode of the translation is " << std::endl << cv::norm(t) << std::endl;
    pose_diff.matrix() = Transf.cast<double>();

    std::cout << "Finish Registration of the Lidar Scans" << std::endl;
}

#endif //MULTIMODALEVNPERCPTION_LIDAR_FRONTEND_NODE_H
