/*
* Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*   * Redistributions of source code must retain the above copyright notice,
*     this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived from
*     this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

// %Tag(FULLTEXT)%

#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Geometry>
#include <Eigen/Core>


#include <deque>
#include <vector>
#include <algorithm>

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

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/sfm/triangulation.hpp>
#include <opencv2/video/tracking.hpp>

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
//#include "/work/tools/g2o/build_a48f/installation/include/g2o/solvers/csparse/linear_solver_csparse.h"
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <chrono>

#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include "Geocentric/LocalCartesian.hpp"
#include "data_type.h"
#include "ba_cost.h"
#include "ba_directedge.h"
#include "frame.h"
#include "mappoint.h"
#include "map.h"
#include "object_det.h"
#include "tracker.h"
#include "pc_frame.h"
#include "imu_frame.h"
#include "gnss_frame.h"

static pcl::PointCloud<pcl::PointXYZ>::Ptr pre_scan_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
static pcl::PointCloud<pcl::PointXYZ>::Ptr curr_scan_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
static pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_scan_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pc_vector;

const int NumThreads = 10;
const double epsilon = 0.1;
const int step = 1;
const double res = 10.0;
const int iter = 60;

// Declaration of Publishers

static ros::Publisher scan_pub;
static ros::Publisher chatter_pub;
static ros::Publisher lidar_odom_pub;
static ros::Publisher gnss_odom_pub;
static ros::Publisher imu_odom_pub;
static ros::Publisher lcam_odom_pub;

// Declaration of Subscribers
static ros::Subscriber scan_sub;
static ros::Subscriber gnss_sub;
static ros::Subscriber imu_sub;
static ros::Subscriber lcam_sub;
static ros::Subscriber lcam_sub_class;

static int count_estimation_lcam = 1;
// Declaration of the rotation
//tf2::Quaternion quat_curr_veh;
//Eigen::Quaternionf curr_q_lidar;

// If the curr_odom already initialized
bool gnss_odom_initialized = false;

// Declaration and Initialization of Vehicle Pose
static Eigen::Matrix4f pose_gnss = Eigen::Matrix4f::Identity();
static Eigen::Matrix4f pose_gnss_pre = Eigen::Matrix4f::Identity();

static double pose_x_gnss;
static double pose_y_gnss;
static double pose_z_gnss;

static double pose_qx_gnss;
static double pose_qy_gnss;
static double pose_qz_gnss;
static double pose_qw_gnss;

static double pose_roll_gnss;
static double pose_pitch_gnss;
static double pose_yaw_gnss;

static Eigen::Isometry3d pose_lidar = Eigen::Isometry3d::Identity();
static Eigen::Isometry3d pose_lidar_pre = Eigen::Isometry3d::Identity();
static Eigen::Isometry3d pose_lidar_diff = Eigen::Isometry3d::Identity();

static double pose_x_lidar;
static double pose_y_lidar;
static double pose_z_lidar;

static double pose_qx_lidar;
static double pose_qy_lidar;
static double pose_qz_lidar;
static double pose_qw_lidar;

static double pose_pitch_lidar;
static double pose_yaw_lidar;
static double pose_roll_lidar;

static Eigen::Quaterniond q_lidar2imu;
static Eigen::Matrix3d rotation_matrix_lidar2imu;

static cv::Mat curr_lcam_img;
static cv::Mat pre_lcam_img;

static double pose_x_lcam;
static double pose_y_lcam;
static double pose_z_lcam;

static double pose_qx_lcam;
static double pose_qy_lcam;
static double pose_qz_lcam;
static double pose_qw_lcam;

static double pose_roll_lcam;
static double pose_pitch_lcam;
static double pose_yaw_lcam;

static Eigen::Isometry3d pose_lcam = Eigen::Isometry3d::Identity();

static Eigen::Matrix3f rotation_matrix_lcam;
static Eigen::Quaterniond q_lcam;

static double stamp_lcam_msg;
static Eigen::Quaterniond q_lcam2imu;
static Eigen::Matrix3d rotation_matrix_lcam2imu;

static int count_lcam_callback = 0;
static bool first_match = true;
static bool first_match_finished = false;
static std::vector<cv::Point2f> kps2f_pre_lcam;
static std::vector<cv::Point2f> kps2f_curr_lcam;
static cv::Mat descptor_curr_lcam;
static cv::Mat descptor_pre_lcam;
static std::vector<cv::KeyPoint> keypoints_pre_lcam;
static std::vector<cv::KeyPoint> keypoints_curr_lcam;
static std::vector<cv::Point3d> points_landmark;
static std::vector<cv::DMatch> matches;
std::shared_ptr<map> map_ptr(new map());

tracker tracker_2d;

static Eigen::Isometry3d pose_imu = Eigen::Isometry3d::Identity();

static double pose_x_imu;
static double pose_y_imu;
static double pose_z_imu;

static double pose_qx_imu;
static double pose_qy_imu;
static double pose_qz_imu;
static double pose_qw_imu;

static double pose_pitch_imu;
static double pose_yaw_imu;
static double pose_roll_imu;

static Eigen::Matrix3f rotation_matrix_imu2imu;
static Eigen::Quaterniond q_imu2imu;

static double stamp_imu_msg;

// Intrinsic Parameter
//static cv::Mat K_lcam = (cv::Mat_<double>(3, 3) << 718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1);
static cv::Mat K_lcam = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

GeographicLib::LocalCartesian geo_converter;
GeographicLib::LocalCartesian gnss_frame::geo_converter_ = GeographicLib::LocalCartesian(0.0, 0.0, 0.0);

static std::deque<point_cloud> point_cloud_ptr_buffer;
static std::deque<gnss_data> gnss_data_buffer;
static std::deque<imu_data> imu_data_buffer;
static std::deque<lcam_data> lcam_data_buffer;

void get_orientation(double gnss_data_time, const std::deque<imu_data>& data_buffer)
{
    imu_data curr_imu;
    curr_imu = data_buffer.front();
    double imu_data_time;

    pose_qx_gnss = curr_imu.orientation_x;
    pose_qy_gnss = curr_imu.orientation_y;
    pose_qz_gnss = curr_imu.orientation_z;
    pose_qw_gnss = curr_imu.orientation_w;

//    if (data_buffer.size() > 1)
//    // If there are more than 1 element in the buffer
//    {
//        //check if the time stamp of gnss_data is newer than the newest from imu_data_buffer
//        curr_imu = data_buffer.front();
//        imu_data_time = curr_imu.time;
//        if (imu_data_time < gnss_data_time)
//        {
//            //update the orientation with the orientation from the newest imu_data
//            pose_qx_gnss = curr_imu.orientation_x;
//            pose_qy_gnss = curr_imu.orientation_y;
//            pose_qz_gnss = curr_imu.orientation_z;
//            pose_qw_gnss = curr_imu.orientation_w;
//        }
//        else
//            //if the time stamp of gnss_data is older than the newest from imu_data_buffer
//        {
//            //todo: interpolate the orientation
//        }
//    }

//    std::cout << "Orientation in x direction is " << pose_qx_gnss << std::endl;



}

bool isRotationMatrix(const cv::Mat& R)
{
    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat ShouldBeIdentity = Rt*R;
    cv::Mat I = cv::Mat::eye(3,3, ShouldBeIdentity.type());
    return  cv::norm(I, ShouldBeIdentity) < 1e-6;
}

void rotationmatrixToEulerAngles(const cv::Mat& R)
{
    assert(isRotationMatrix(R));
    float sy = sqrt(R.at<double>(0, 0)*R.at<double>(0, 0) + R.at<double>(1, 0)*R.at<double>(1, 0));
    bool singular = sy < 1e-6;
    float x, y, z; // yaw, pitch, roll
    if(!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    std::cout << "yaw is " << 180.0*x/3.1415 << std::endl
              << "pitch is " << 180.0*y/3.1415 << std::endl
              << "roll is " << 180.0*z/3.1415 << std::endl;
    return;
}



void img_store(const cv::Mat& img, const int type)
{
//    // Draw the image with matches
//    cv::Mat img_match;
//    std::cout<<"Size of the keypoints, pre and curr are " << keypoints_pre_lcam.size() <<" "<< keypoints_curr_lcam.size() << " " << matches.size() << std::endl;
//    cv::drawMatches(pre_lcam_img, keypoints_pre_lcam, curr_lcam_img, keypoints_curr_lcam, matches, img_match);
    // Store the images ////////////////////////////////////////////////////////////////////////////////
    if (type == 0)
    {
        std::string image_file1("/work/catkin_ws_ndt/src/localization_ndt/img/img_frame_with_matches_");
        std::stringstream str_stream;
        str_stream << count_lcam_callback;
        std::string image_file2 = str_stream.str();
        std::string image_file3(".png");
        std::string image_file = image_file1 + image_file2 + image_file3;
        cv::imwrite(image_file, img);
    }
    else
    {
        std::string image_file1("/work/catkin_ws_ndt/src/localization_ndt/img/img_frame_with_LK_");
        std::stringstream str_stream;
        str_stream << count_lcam_callback;
        std::string image_file2 = str_stream.str();
        std::string image_file3(".png");
        std::string image_file = image_file1 + image_file2 + image_file3;
        cv::imwrite(image_file, img);
    }
}

void find_feature_matches ( const cv::Mat& img_1, const cv::Mat& img_2)
{
    //-- 初始化
    cv::Mat descriptors_1, descriptors_2;
    // used in OpenCV3
    // todo: Consider using SIFT Features
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();

    cv::Ptr<cv::DescriptorMatcher> matcher  = cv::DescriptorMatcher::create ( "BruteForce-Hamming" );
    //-- 第一步:检测 Oriented FAST 角点位置
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    std::vector<cv::DMatch> match;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, match );

    // Draw the image with matches
    cv::Mat img_match;
    std::cout<<"Size of the keypoints, pre and curr are " << keypoints_1.size() <<" "<< keypoints_2.size() << " " << match.size() << std::endl;
    cv::drawMatches(pre_lcam_img, keypoints_1, curr_lcam_img, keypoints_2, match, img_match);
//    cv::imshow ( "匹配点对", img_match );
//    cv::waitKey(0);

    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    // 同时更新匹配和特征点
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( match[i].distance <= std::max ( 2*min_dist, 30.0 ) )
        {
            matches.push_back ( match[i] );
            keypoints_pre_lcam.push_back (keypoints_1[match[i].queryIdx]);
            keypoints_curr_lcam.push_back (keypoints_2[match[i].trainIdx]);
            kps2f_pre_lcam.push_back (keypoints_1[match[i].queryIdx].pt);
            kps2f_curr_lcam.push_back ( keypoints_2[match[i].trainIdx].pt);
        }
    }

    cv::drawMatches(pre_lcam_img, keypoints_1, curr_lcam_img, keypoints_2, matches, img_match);
    img_store(img_match, 0);
    // 重新根据筛选后的角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_pre_lcam, descptor_pre_lcam );
    descriptor->compute ( img_2, keypoints_curr_lcam, descptor_pre_lcam );

    std::cout << "number of keypoints and descriptors are " << keypoints_pre_lcam.size() << " " << descptor_pre_lcam.rows << std::endl;
//    cv::imshow ( "优化后匹配点对", img_match );
//    cv::waitKey(0);
}

void ba_2d2d_direct_ceres(
        const cv::Mat& R_initial_cv,
        const cv::Mat& t_initial_cv
        )
{
    // Get initial guess ///////////////////////////////////////////////////////////////////////////////////////////

    // initial guess for translation vector
    double t_initial[3];
    t_initial[0] = t_initial_cv.at<double>(0, 0);
    t_initial[1] = t_initial_cv.at<double>(1, 0);
    t_initial[2] = t_initial_cv.at<double>(2, 0);
//    std::cout << "reached line 430" << std::endl;
    // initial guess for rotation matrix
    Eigen::Matrix3d R_initial_eigen;
    cv2eigen(R_initial_cv, R_initial_eigen);
    // convert to quaternion
    double q_initial[4];
    Eigen::Quaterniond q_initial_eigen = Eigen::Quaterniond(R_initial_eigen);
    // coeffs: x, y, z, w.
    // ceres w, x, y, z
    q_initial[0] = q_initial_eigen.coeffs()[3]; //w
    q_initial[1] = q_initial_eigen.coeffs()[0]; //x
    q_initial[2] = q_initial_eigen.coeffs()[1]; //y
    q_initial[3] = q_initial_eigen.coeffs()[2]; //z
    // convert to angle axis, which is used in ceres
    double angleaxis_initial[3];
    ceres::QuaternionToAngleAxis(q_initial, angleaxis_initial);
//    std::cout << "reached line 446" << std::endl;
    // Transform, angle axis [3] and t[3]
    double T[6];
    T[0] = angleaxis_initial[0];
    T[1] = angleaxis_initial[1];
    T[2] = angleaxis_initial[2];
    T[3] = t_initial[0];
    T[4] = t_initial[1];
    T[5] = t_initial[2];
//    std::cout << "reached line 455" << std::endl;
    // Problem Building /////////////////////////////////////////////////////////////////////////////////////////////////////
    ceres::Problem ba_problem;
    // Output residual: 1D;
    // Input Parameter Block Nr. 1: 6D;
    // Input Parameter Block Nr. 2: 3D;

    // Ordering for marginalization
    // element in group 0 comes with higher prioritiy in marginalization
    ceres::ParameterBlockOrdering* ordering = new ceres::ParameterBlockOrdering;
    ordering->AddElementToGroup(T, 1);

    // Adding Residual Block to the problem
//    std::cout << "reached line 468" << std::endl;
    // Vector of pointers of all landmark points
    std::vector<double*> ptr_Pts;
//    Eigen::Matrix<double, pre_lcam_img.rows, pre_lcam_img.cols> pre_lcam_img_eigen, curr_lcam_img_eigen;
//    cv2eigen(pre_lcam_img, pre_lcam_img_eigen);
//    cv2eigen(curr_lcam_img, curr_lcam_img_eigen);
    // There ought to be one residual block for one observation
    for (int i = 0; i < pre_lcam_img.rows; i++)
    {
        for (int j = 0; j < pre_lcam_img.cols; j++)
        {
//            std::cout << "reached line 479" << std::endl;
            // Pointer to landmark point which comes with the initial guess
            // Note：此处一定要使用new来新建数组，因为new会给这个变量重新分配内存，如果不使用new，local_ptr的地址在这个循环中一直保持不变，实际上就会没有给这个优化问题创建新的优化变量
            double* local_Pt = new double[3]{1.0, 1.0, 1.0};
            // Push back the pointer
            ptr_Pts.push_back(local_Pt);
            // todo: consider using factory construction function
//            ceres::CostFunction* ba_cost_direct = new ceres::AutoDiffCostFunction<BA_DIRECT_COST, 1, 6, 3>(new BA_DIRECT_COST(double(i), double(j), pre_lcam_img_eigen, curr_lcam_img_eigen));
            ceres::CostFunction* ba_cost_direct = new ceres::NumericDiffCostFunction<BA_DIRECT_COST, ceres::FORWARD, 1, 6, 3>(new BA_DIRECT_COST(double(i), double(j), pre_lcam_img, curr_lcam_img));
            // 传递进这个Residual Block的是每个Obseration对应的待优化变量的指针。传递数组本身也就相当于传递了数组的指针。
            ba_problem.AddResidualBlock(ba_cost_direct, nullptr, T, local_Pt);
            // Prioritize landmark points in marginalization
            ordering->AddElementToGroup(local_Pt, 0);
        }
    }

    std::cout << "Problem Building Finished" << std::endl;

    ceres::Solver::Options ba_options;
    ba_options.max_num_iterations = 30;
//    ba_options.linear_solver_type = ceres::DENSE_QR;
    ba_options.linear_solver_type = ceres::SPARSE_SCHUR;
    ba_options.linear_solver_ordering.reset(ordering);
    ba_options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary ba_summary;
    ceres::Solve(ba_options, &ba_problem, &ba_summary);
    std::cout << ba_summary.BriefReport() << "\n";
    std::cout << "reached line 507" << std::endl;

    // Postprocessing to get the optimized T and Pts //////////////////////////////////////////////////////////////////////////////////////////////
    double angleaxis_opt[3] = {T[0], T[1], T[2]}; // Optimized Angle Axis, consisting of a normal vector, representing the rotation axis and rotation angle
    double translate_opt[3] = {T[3], T[4], T[5]}; // Optimized Translation

    double q_opt[4];

    ceres::AngleAxisToQuaternion(angleaxis_opt, q_opt); // quaternion: w, x, y, z
    Eigen::Quaterniond q_opt_eigen = Eigen::Quaterniond(q_opt[0], q_opt[1], q_opt[2], q_opt[3]); // Oder: w, x, y, z
    q_opt_eigen.normalize();

    // Construct a Rotation Matrix in Eigen
    Eigen::Isometry3d T_eigen = Eigen::Isometry3d::Identity();
    T_eigen.rotate(q_opt_eigen);
    T_eigen.pretranslate(Eigen::Vector3d(translate_opt[0], translate_opt[1], translate_opt[2]));
    std::cout << "Direct Method: camera pose after optimization is " <<  std::endl << T_eigen.matrix() << std::endl;

    // Pointer to the first element in the array of landmark point
    std::cout << "Direct Method: landmark point pointer after optimization is " <<  std::endl << ptr_Pts[0] << std::endl;
    // Pointer to the third element in the array of landmark point, which is the z (depth)
    std::cout << "Direct Method: depth of the landmark point after optimization is " <<  std::endl << *(ptr_Pts[0] + 2) << std::endl;

}

void pose_estimation_2d2d(
        cv::Mat& R,
        cv::Mat& t)
{
    //-- 计算本质矩阵, default is RANSAC, the t vector is already normalized to 1
    cv::Mat essential_matrix;
    essential_matrix = cv::findEssentialMat(kps2f_pre_lcam, kps2f_curr_lcam, K_lcam);
    std::cout << "essential_matrix is "<< std::endl << essential_matrix << std::endl;

    //-- 从本质矩阵中恢复旋转和平移信息.
    // todo: check if normalization of t is done
    cv::recoverPose(essential_matrix, kps2f_pre_lcam, kps2f_curr_lcam, K_lcam, R, t);
    std::cout << "R is " << std::endl << R << std::endl;
    std::cout << "t is " << std::endl << t << std::endl;
    std::cout << "mode of t is " << cv::norm(t) << std::endl;

//    // From Gao //////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//    std::vector<cv::Point2f> points1;
//    std::vector<cv::Point2f> points2;
//
//    for ( int i = 0; i < ( int ) matches.size(); i++ )
//    {
//        points1.push_back ( keypoints_pre_lcam[matches[i].queryIdx].pt );
//        points2.push_back ( keypoints_curr_lcam[matches[i].trainIdx].pt );
//    }
//
//
//    //-- 计算本质矩阵, default is RANSAC, the t vector is already normalized to 1
//    cv::Mat essential_matrix_Gao;
//    essential_matrix_Gao = cv::findEssentialMat(points1, points2, K_lcam);
//    std::cout << "essential_matrix_Gao is "<< std::endl << essential_matrix_Gao << std::endl;
//
//    //-- 从本质矩阵中恢复旋转和平移信息.
//    // todo: check if normalization of t is done
//    cv::Mat R_Gao, t_Gao;
//    cv::recoverPose(essential_matrix_Gao, points1, points2, K_lcam, R_Gao, t_Gao);
//    std::cout << "R_Gao is " << std::endl << R_Gao << std::endl;
//    std::cout << "t_Gao is " << std::endl << t_Gao << std::endl;
//    std::cout << "mode of t_Gao is " << cv::norm(t_Gao) << std::endl;

}

cv::Point2f pixel2cam ( const cv::Point2f& p, const cv::Mat& K )
{
    return cv::Point2f
            (
                    ( p.x - K.at<double>(0,2) ) / K.at<double>(0,0),
                    ( p.y - K.at<double>(1,2) ) / K.at<double>(1,1)
            );
}



void env_update(const std::vector<unsigned char> status)
{
    std::cout<<"Size of the keypoints, 2f, landmark points, descriptor before erase are " << keypoints_pre_lcam.size() <<" "<< kps2f_pre_lcam.size() << " " << points_landmark.size() << " " << descptor_pre_lcam.rows << std::endl;
    int i = 0;
    for ( auto iter=kps2f_pre_lcam.begin(); iter!=kps2f_pre_lcam.end(); i++)
    {
        if ( status[i] == 0 )
        {
            iter = kps2f_pre_lcam.erase(iter);
            continue;
        }
        iter++;
    }
    i = 0;
    for ( auto iter=kps2f_curr_lcam.begin(); iter!=kps2f_curr_lcam.end(); i++)
    {
        if ( status[i] == 0 )
        {
            iter = kps2f_curr_lcam.erase(iter);
            continue;
        }
        iter++;
    }
    i = 0;
    for ( auto iter=points_landmark.begin(); iter!=points_landmark.end(); i++)
    {
        if ( status[i] == 0 )
        {
            iter = points_landmark.erase(iter);
            continue;
        }
        iter++;
    }
//    i = 0;
//    for ( auto iter=keypoints_curr_lcam.begin(); iter!=keypoints_curr_lcam.end(); i++)
//    {
//        if ( status[i] == 0 )
//        {
//            iter = keypoints_curr_lcam.erase(iter);
//            continue;
//        }
//        iter++;
//    }
    i = 0;
    for ( auto iter=keypoints_pre_lcam.begin(); iter!=keypoints_pre_lcam.end(); i++)
    {
        if ( status[i] == 0 )
        {
            iter = keypoints_pre_lcam.erase(iter);
            continue;
        }
        iter++;
    }
//    i = 0;
//    for ( auto iter=matches.begin(); iter!=matches.end(); i++)
//    {
//        if ( status[i] == 0 )
//        {
//            iter = matches.erase(iter);
//            continue;
//        }
//        iter++;
//    }
    cv::Ptr <cv::DescriptorExtractor> descriptor = cv::ORB::create();
    descriptor->compute(pre_lcam_img, keypoints_pre_lcam, descptor_pre_lcam);
    descriptor->compute(curr_lcam_img, keypoints_curr_lcam, descptor_curr_lcam);
    std::cout<<"Size of the keypoints, 2f, landmark points, descriptor after erase are " << keypoints_pre_lcam.size() <<" "<< kps2f_pre_lcam.size() << " " << points_landmark.size() << " " << descptor_pre_lcam.rows << std::endl;
}

void triangulation (
        const std::vector< cv::KeyPoint >& keypoint_1,
        const std::vector< cv::KeyPoint >& keypoint_2,
        const std::vector< cv::DMatch >& matches,
        const cv::Mat& R, const cv::Mat& t,
        std::vector< cv::Point3d >& points )
{
    // 三角化的输入是相对运动，第一张图用零点和无旋转没问题
    // todo:T1和T2需要左乘内参吗
    cv::Mat T1 = (cv::Mat_<float> (3,4) <<
                                1,0,0,0,
            0,1,0,0,
            0,0,1,0);
    cv::Mat T2 = (cv::Mat_<float> (3,4) <<
                                R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
            R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
            R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0)
    );

    std::vector<cv::Point2f> pts_1, pts_2;
    // 三角化函数的输入是相机坐标下的特征点
    // pts_1和pts_2是特征点在各自相机坐标下的坐标值，通过内参计算（pixel2cam),实际上是归一化平面上的坐标
    for ( cv::DMatch m:matches )
    {
        // 将像素坐标转换至相机坐标
        pts_1.push_back ( pixel2cam( keypoint_1[m.queryIdx].pt, K_lcam) );
        pts_2.push_back ( pixel2cam( keypoint_2[m.trainIdx].pt, K_lcam) );
    }

    // 当T1和T2只考虑相对运动时，pts_4d表示的是T1坐标下的landmark奇次坐标
    cv::Mat pts_4d;
    cv::triangulatePoints( T1, T2, pts_1, pts_2, pts_4d );


    // 转换成非齐次坐标
    // todo: 为什么要改成非齐次
    for ( int i=0; i<pts_4d.cols; i++ )
    {
        cv::Mat x = pts_4d.col(i);
        x /= x.at<float>(3,0); // 归一化
        cv::Point3d p (
                x.at<float>(0,0),
                x.at<float>(1,0),
                x.at<float>(2,0)
        );
        points.push_back( p );
    }
}

void triangulation_2f_abs(
        const cv::Mat& R,
        const cv::Mat& t)
{
    //todo: check if the current transform should be used?
    cv::Mat T1 = (cv::Mat_<float> (3,4) <<
                                        1,0,0,0,
            0,1,0,0,
            0,0,1,0);
//    cv::Mat P2 = (cv::Mat_<float> (3,4) <<
//                                        T2.at<double>(0,0), T2.at<double>(0,1), T2.at<double>(0,2), T2.at<double>(0,3),
//            T2.at<double>(1,0), T2.at<double>(1,1), T2.at<double>(1,2), T2.at<double>(1,3),
//            T2.at<double>(2,0), T2.at<double>(2,1), T2.at<double>(2,2), T2.at<double>(2,3));

    cv::Mat T2 = (cv::Mat_<float> (3,4) <<
                                        R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
            R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
            R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0));

    std::vector<cv::Point2f> pts_1, pts_2;
    for (int i = 0; i < kps2f_pre_lcam.size(); i++)
    {
        // 将像素坐标转换至相机坐标/实际上是归一化平面上的坐标
        pts_1.push_back ( pixel2cam( kps2f_pre_lcam[i], K_lcam) );
        pts_2.push_back ( pixel2cam( kps2f_curr_lcam[i], K_lcam) );
    }

    cv::Mat pts_4d;
    // todo: should P1 be projection matrix with calibration matrix
    // Projection Matrix

    cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);
    // 转换成非齐次坐标
    // todo: 为什么要改成齐次
    points_landmark.clear();
    std::vector<unsigned char> status;

    for ( int i=0; i<pts_4d.cols; i++ )
    {
        cv::Mat x = pts_4d.col(i);
        x /= x.at<float>(3,0); // 归一化
        cv::Point3d p (
                x.at<float>(0,0),
                x.at<float>(1,0),
                x.at<float>(2,0)
        );
        if(p.z < 0) // when the depth is negative
        {
            status.push_back(0); // set the status to 0
        }
        else
        {
            status.push_back(1); // set the status to 1
        }
        points_landmark.push_back( p );
    }
    env_update(status); // update the environment (keypoint, landmark, descriptor)
}

void triangulation_2f_rel(
        const cv::Mat& R,
        const cv::Mat& t)
{
    cv::Mat P1 = (cv::Mat_<double> (3,4) <<
                                        1,0,0,0,
            0,1,0,0,
            0,0,1,0);
    cv::Mat P2 = (cv::Mat_<double> (3,4) <<
                                        R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
            R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
            R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0)
    );

    std::vector<cv::Point2f> pts_1, pts_2;
    for (int i = 0; i < kps2f_pre_lcam.size(); i++)
    {
        // 将像素坐标转换至相机坐标/实际上是归一化平面上的坐标
        pts_1.push_back ( pixel2cam( kps2f_pre_lcam[i], K_lcam) );
        pts_2.push_back ( pixel2cam( kps2f_curr_lcam[i], K_lcam) );
    }
//    P1 = K_lcam*P1;
//    P2 = K_lcam*P2;

    cv::Mat pts_4d; // pts_4d是在P1坐标系下的landmark的非齐次坐标
    cv::triangulatePoints(P1, P2, pts_1, pts_2, pts_4d);
    // 转换成非齐次坐标
    // todo: 为什么要改成齐次
    points_landmark.clear();
    std::vector<unsigned char> status;

    for ( int i=0; i<pts_4d.cols; i++ )
    {
        cv::Mat x = pts_4d.col(i);
        x /= x.at<float>(3,0); // 归一化
        cv::Point3d p (
                x.at<float>(0,0),
                x.at<float>(1,0),
                x.at<float>(2,0)
        );
        if(p.z < 0) // when the depth is negative
        {
            status.push_back(0); // set the status to 0
        }
        else
        {
            status.push_back(1); // set the status to 1
        }
        points_landmark.push_back( p );
    }
    env_update(status); // update the environment (keypoint, landmark, descriptor)
}

void triangulation_2f(
        const cv::Mat& T1,
        const cv::Mat& T2)
{
//    //todo: check if the current transform should be used?
//    cv::Mat T1 = (cv::Mat_<float> (3,4) <<
//                                        1,0,0,0,
//            0,1,0,0,
//            0,0,1,0);
//    cv::Mat T2 = (cv::Mat_<float> (3,4) <<
//                                        R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
//            R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
//            R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0)
//    );

    std::vector<cv::Point2f> pts_1, pts_2;
    for (int i = 0; i < kps2f_pre_lcam.size(); i++)
    {
        // 将像素坐标转换至相机坐标/实际上是归一化平面上的坐标
        pts_1.push_back ( pixel2cam( kps2f_pre_lcam[i], K_lcam) );
        pts_2.push_back ( pixel2cam( kps2f_curr_lcam[i], K_lcam) );
    }

    cv::Mat pts_4d;
    // todo: should P1 be projection matrix with calibration matrix
    // Projection Matrix
//    cv::Mat P1, P2;
    cv::Mat P1 = (cv::Mat_<float> (3,4) <<
                                        T1.at<double>(0,0), T1.at<double>(0,1), T1.at<double>(0,2), T1.at<double>(0,3),
            T1.at<double>(1,0), T1.at<double>(1,1), T1.at<double>(1,2), T1.at<double>(1,3),
            T1.at<double>(2,0), T1.at<double>(2,1), T1.at<double>(2,2), T1.at<double>(2,3));
    cv::Mat P2 = (cv::Mat_<float> (3,4) <<
                                        T2.at<double>(0,0), T2.at<double>(0,1), T2.at<double>(0,2), T2.at<double>(0,3),
            T2.at<double>(1,0), T2.at<double>(1,1), T2.at<double>(1,2), T2.at<double>(1,3),
            T2.at<double>(2,0), T2.at<double>(2,1), T2.at<double>(2,2), T2.at<double>(2,3));

    cv::triangulatePoints(P1, P2, pts_1, pts_2, pts_4d);
    // 转换成非齐次坐标
    // todo: 为什么要改成齐次
    points_landmark.clear();
    std::vector<unsigned char> status;

    for ( int i=0; i<pts_4d.cols; i++ )
    {
        cv::Mat x = pts_4d.col(i);
        x /= x.at<float>(3,0); // 归一化
        cv::Point3d p (
                x.at<float>(0,0),
                x.at<float>(1,0),
                x.at<float>(2,0)
        );
        if(p.z < 0) // when the depth is negative
        {
            status.push_back(0); // set the status to 0
        }
        else
        {
            status.push_back(1); // set the status to 1
        }
        points_landmark.push_back( p );
    }
    env_update(status); // update the environment (keypoint, landmark, descriptor)
}

void pose_estimation_3d2d(
        const std::vector< cv::KeyPoint >& keypoints_1,
        const std::vector< cv::KeyPoint >& keypoints_2,
        const std::vector<cv::Point3d>& points_3d,
        const std::vector< cv::DMatch >& matches,
        cv::Mat& R, cv::Mat& t
        )
{
    std::vector<cv::Point2f> points_1, points_2;
    //points_2 should be the key point in the camera frame
    for (int i = 0; i < matches.size(); i++)
    {
        points_1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points_2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }

    cv::Mat r;
    cv::solvePnP( points_3d, points_2, K_lcam, cv::Mat(), r, t, false ); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
    cv::Rodrigues ( r, R ); // r为旋转向量形式，用Rodrigues公式转换为矩阵
}

void pose_estimation_3d2d_2f(
        cv::Mat& R,
        cv::Mat& t
)
{
    cv::Mat r;
    std::cout << "3D-2D Matching Started" << std::endl
              <<"size of keypoints and landmark points are "
              << std::endl << points_landmark.size()
              << std::endl << kps2f_curr_lcam.size() << std::endl;
    // points_3d is the landmark point in the first camera frame
    // todo: use ransac pnp
    cv::solvePnP(points_landmark, kps2f_curr_lcam, K_lcam, cv::Mat(), r, t, false ); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
    cv::Rodrigues ( r, R ); // r为旋转向量形式，用Rodrigues公式转换为矩阵
}

void ba_3d2d_g2o (
        const std::vector< cv::Point3d > points_3d,
        const std::vector<cv::Point2f>& points_2f,
        const cv::Mat& R,
        const cv::Mat& t,
        cv::Mat& R_opt,
        cv::Mat& t_opt,
        std::vector<cv::Point3d>& points_3d_opt)
{
    // 初始化g2o
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose 维度为 6, landmark 维度为 3
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); // 线性方程求解器
//    Block* solver_ptr = new Block ( linearSolver );
    Block* solver_ptr = new Block( std::unique_ptr<Block::LinearSolverType>(linearSolver) );// 矩阵块求解器// 矩阵块求解器
//    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<Block>(solver_ptr) );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );

    // vertex
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    Eigen::Matrix3d R_mat;
    R_mat <<
          R.at<double> ( 0,0 ), R.at<double> ( 0,1 ), R.at<double> ( 0,2 ),
            R.at<double> ( 1,0 ), R.at<double> ( 1,1 ), R.at<double> ( 1,2 ),
            R.at<double> ( 2,0 ), R.at<double> ( 2,1 ), R.at<double> ( 2,2 );

    pose->setId ( 0 );
    pose->setEstimate ( g2o::SE3Quat (
            R_mat,
            Eigen::Vector3d ( t.at<double> ( 0,0 ), t.at<double> ( 1,0 ), t.at<double> ( 2,0 ) )
    ) );
    optimizer.addVertex ( pose );

    int index = 1;
    for ( const cv::Point3d p:points_3d )   // landmarks
    {
        g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
        point->setId (index++);
        point->setEstimate ( Eigen::Vector3d ( p.x, p.y, p.z ) );
        point->setMarginalized ( true ); // g2o 中必须设置 marg 参见第十讲内容
        optimizer.addVertex ( point );
//        index++;
    }

    // parameter: camera intrinsics
    g2o::CameraParameters* camera = new g2o::CameraParameters (
            K_lcam.at<double> ( 0,0 ), Eigen::Vector2d ( K_lcam.at<double> ( 0,2 ), K_lcam.at<double> ( 1,2 ) ), 0
    );
    camera->setId ( 0 );
    optimizer.addParameter ( camera );

    // edges
//    std::vector<cv::Point2f> points_2f;
//    for (int i = 0; i < matches.size(); i++)
//    {
//        points_2f.push_back(keypoints_2[matches[i].trainIdx].pt);
//    }
    index = 1;
    for ( const cv::Point2f p:points_2f )
    {
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId ( index );
        edge->setVertex ( 0, dynamic_cast<g2o::VertexSBAPointXYZ*> ( optimizer.vertex ( index ) ) );
        edge->setVertex ( 1, pose );
        edge->setMeasurement ( Eigen::Vector2d ( p.x, p.y ) );
        edge->setParameterId ( 0,0 );
        edge->setInformation ( Eigen::Matrix2d::Identity() );
        optimizer.addEdge ( edge );
        index++;
    }

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    optimizer.setVerbose ( true );
    optimizer.initializeOptimization();
    optimizer.optimize ( 100 );
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>> ( t2-t1 );
    std::cout<<"optimization costs time: "<<time_used.count() <<" seconds."<<std::endl;

    std::cout<<std::endl<<"after optimization using g2o:"<<std::endl;
    std::cout<<"T="<<std::endl<<Eigen::Isometry3d ( pose->estimate() ).matrix() <<std::endl;
    Eigen::Matrix4d T_mat = Eigen::Isometry3d ( pose->estimate() ).matrix();
    R_opt = (cv::Mat_<double> (3, 3) <<
            T_mat(0, 0), T_mat(0, 1), T_mat(0, 2),
            T_mat(1, 0), T_mat(1, 1), T_mat(1, 2),
            T_mat(2, 0), T_mat(2, 1), T_mat(2, 2));
    t_opt = (cv::Mat_<double> (3, 1) << T_mat(0, 3), T_mat(1, 3), T_mat(2, 3));

    Eigen::Vector3d point_3d_opt;
    cv::Point3d point_3d_opt_cv;
    index = 1;
    for ( const cv::Point3d p:points_3d )   // landmarks
    {
        point_3d_opt = dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(index))->estimate();
        point_3d_opt_cv.x = point_3d_opt(0);
        point_3d_opt_cv.y = point_3d_opt(1);
        point_3d_opt_cv.z = point_3d_opt(2);
        points_3d_opt.push_back(point_3d_opt_cv);
        index++;
    }


}

void new_kp_add() {
    std::cout << "Adding New Feature Started" << std::endl;
    std::cout << "Number of keypoints(2f), keypoints and descriptors are " << kps2f_pre_lcam.size() << " "
              << keypoints_pre_lcam.size() << " " << descptor_pre_lcam.rows << std::endl;

    // 2D-2D Feature Matching to generate proposal of key points
    std::vector <cv::KeyPoint> keypoints_pre_lcam_new, keypoints_curr_lcam_new;
    cv::Mat descptor_pre_lcam_new, descptor_curr_lcam_new;
    std::vector <cv::DMatch> matches_new;

    //-- 初始化
    // used in OpenCV3
    cv::Ptr <cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr <cv::DescriptorExtractor> descriptor = cv::ORB::create();

    //-- 第一步:检测 Oriented FAST 角点位置

    detector->detect(pre_lcam_img, keypoints_pre_lcam_new);
    detector->detect(curr_lcam_img, keypoints_curr_lcam_new);

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute(pre_lcam_img, keypoints_pre_lcam_new, descptor_pre_lcam_new);
    descriptor->compute(curr_lcam_img, keypoints_curr_lcam_new, descptor_curr_lcam_new);

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    cv::Ptr <cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
//    std::vector<cv::DMatch> match;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match(descptor_pre_lcam_new, descptor_curr_lcam_new, matches_new);

    //-- 第四步:匹配点对筛选
    double min_dist_new = 10000, max_dist_new = 0;
    std::cout << "Number of new keypoints and descriptors are " << " " << keypoints_pre_lcam_new.size() << " "
              << descptor_pre_lcam_new.rows << std::endl;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for (int i = 0; i < descptor_pre_lcam_new.rows; i++) {
        double dist_new = matches_new[i].distance;
        if (dist_new < min_dist_new) min_dist_new = dist_new;
        if (dist_new > max_dist_new) max_dist_new = dist_new;
    }

    printf("-- Max dist new : %f \n", max_dist_new);
    printf("-- Min dist new : %f \n", min_dist_new);

    double min_dist = 10000, max_dist = 0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for (int i = 0; i < descptor_pre_lcam.rows; i++) {
        double dist = matches[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);

    min_dist = std::min(min_dist, min_dist_new);

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    // 同时更新匹配和特征点
    for (int i = 0; i < descptor_pre_lcam_new.rows; i++) {
        if (matches_new[i].distance <= std::max(2 * min_dist, 30.0)) {
            matches.push_back ( matches_new[i] );
            keypoints_pre_lcam.push_back(keypoints_pre_lcam_new[matches_new[i].queryIdx]);
            keypoints_curr_lcam.push_back(keypoints_curr_lcam_new[matches_new[i].trainIdx]);
            kps2f_pre_lcam.push_back(keypoints_pre_lcam_new[matches_new[i].queryIdx].pt);
            kps2f_curr_lcam.push_back(keypoints_curr_lcam_new[matches_new[i].trainIdx].pt);
        }
    }
    // 重新根据筛选后的角点位置计算 BRIEF 描述子
    descriptor->compute(pre_lcam_img, keypoints_pre_lcam, descptor_pre_lcam);
    descriptor->compute(curr_lcam_img, keypoints_curr_lcam, descptor_curr_lcam);
    std::cout << "Adding New Feature " << std::endl;
    std::cout << "Number of New keypoints(2f), keypoints and descriptors are " << kps2f_pre_lcam.size() << " "
              << keypoints_pre_lcam.size() << " " << descptor_pre_lcam.rows << std::endl;

    // 根据新的特征点作匹配，使用hamming距离
    std::vector< cv::DMatch > matches_tmp;
    matcher->match(descptor_pre_lcam, descptor_curr_lcam, matches_tmp);
    matches.clear();
    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    std::vector< cv::KeyPoint > keypoints_pre_lcam_tmp;
    std::vector< cv::KeyPoint > keypoints_curr_lcam_tmp;
    keypoints_pre_lcam_tmp.swap(keypoints_pre_lcam);
    keypoints_curr_lcam_tmp.swap(keypoints_curr_lcam);
    keypoints_pre_lcam.clear();
    keypoints_curr_lcam.clear();
    kps2f_pre_lcam.clear();
    kps2f_curr_lcam.clear();
    for ( int i = 0; i < descptor_curr_lcam.rows; i++ )
    {
        if ( matches_tmp[i].distance <= std::max ( 2*min_dist, 30.0 ) )
        {
            matches.push_back ( matches_tmp[i] );
            keypoints_pre_lcam.push_back(keypoints_pre_lcam_tmp[matches_tmp[i].queryIdx]);
            keypoints_curr_lcam.push_back(keypoints_curr_lcam_tmp[matches_tmp[i].trainIdx]);
            kps2f_pre_lcam.push_back(keypoints_pre_lcam_tmp[matches_tmp[i].queryIdx].pt);
            kps2f_curr_lcam.push_back(keypoints_curr_lcam_tmp[matches_tmp[i].trainIdx].pt);
        }
    }
    // 重新根据筛选后的角点位置计算 BRIEF 描述子
    descriptor->compute(pre_lcam_img, keypoints_pre_lcam, descptor_pre_lcam);
    descriptor->compute(curr_lcam_img, keypoints_curr_lcam, descptor_curr_lcam);

    std::cout << "Adding New Feature Finished" << std::endl;
    std::cout << "Number of keypoints(2f), keypoints and descriptors are " << kps2f_pre_lcam.size() << " "
              << keypoints_pre_lcam.size() << " " << descptor_pre_lcam.rows << std::endl;

}

void brandnew_kp_add() {
    std::cout << "Adding New Feature Started" << std::endl;
    std::cout << "Number of keypoints(2f), keypoints and descriptors are " << kps2f_pre_lcam.size() << " "
              << keypoints_pre_lcam.size() << " " << descptor_pre_lcam.rows << std::endl;

    // 2D-2D Feature Matching to generate proposal of key points
    std::vector <cv::KeyPoint> keypoints_pre_lcam_new, keypoints_curr_lcam_new;
    cv::Mat descptor_pre_lcam_new, descptor_curr_lcam_new;
    std::vector <cv::DMatch> matches_new;

    //-- 初始化
    // used in OpenCV3
    cv::Ptr <cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr <cv::DescriptorExtractor> descriptor = cv::ORB::create();

    //-- 第一步:检测 Oriented FAST 角点位置

    detector->detect(pre_lcam_img, keypoints_pre_lcam_new);
    detector->detect(curr_lcam_img, keypoints_curr_lcam_new);

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute(pre_lcam_img, keypoints_pre_lcam_new, descptor_pre_lcam_new);
    descriptor->compute(curr_lcam_img, keypoints_curr_lcam_new, descptor_curr_lcam_new);

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    cv::Ptr <cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
//    std::vector<cv::DMatch> match;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match(descptor_pre_lcam_new, descptor_curr_lcam_new, matches_new);

    //-- 第四步:匹配点对筛选
    double min_dist_new = 10000, max_dist_new = 0;
    std::cout << "Number of new keypoints and descriptors are " << " " << keypoints_pre_lcam_new.size() << " "
              << descptor_pre_lcam_new.rows << std::endl;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for (int i = 0; i < descptor_pre_lcam_new.rows; i++) {
        double dist_new = matches_new[i].distance;
        if (dist_new < min_dist_new) min_dist_new = dist_new;
        if (dist_new > max_dist_new) max_dist_new = dist_new;
    }

    printf("-- Max dist new : %f \n", max_dist_new);
    printf("-- Min dist new : %f \n", min_dist_new);

    keypoints_pre_lcam.clear();
    keypoints_curr_lcam.clear();
    kps2f_pre_lcam.clear();
    kps2f_curr_lcam.clear();
    matches.clear();

    for ( int i = 0; i < descptor_curr_lcam_new.rows; i++ )
    {
        if ( matches_new[i].distance <= std::max ( 2*min_dist_new, 30.0 ) )
        {
            matches.push_back ( matches_new[i] );
            keypoints_pre_lcam.push_back(keypoints_pre_lcam_new[matches_new[i].queryIdx]);
            keypoints_curr_lcam.push_back(keypoints_curr_lcam_new[matches_new[i].trainIdx]);
            kps2f_pre_lcam.push_back(keypoints_pre_lcam_new[matches_new[i].queryIdx].pt);
            kps2f_curr_lcam.push_back(keypoints_curr_lcam_new[matches_new[i].trainIdx].pt);
        }
    }

    cv::Mat img_match;
    std::cout<<"Size of the keypoints, pre and curr are after adding new feature" << keypoints_pre_lcam.size() <<" "<< keypoints_pre_lcam.size() << " " << matches.size() << std::endl;
    cv::drawMatches(pre_lcam_img, keypoints_pre_lcam_new, curr_lcam_img, keypoints_curr_lcam_new, matches, img_match);
    img_store(img_match, 0);

}

void triangulation_verify(
        const cv::Mat& R,
        const cv::Mat& t)
{

    for ( int i=0; i<points_landmark.size(); i++ )
    {
        cv::Point2d pt1_cam = pixel2cam(kps2f_pre_lcam[i], K_lcam );
        cv::Point2d pt1_cam_3d(
                points_landmark[i].x/points_landmark[i].z,
                points_landmark[i].y/points_landmark[i].z
        );

        std::cout<<"point in the first camera frame: "<<pt1_cam<<std::endl;
        std::cout<<"point projected from 3D "<<pt1_cam_3d<<", d="<<points_landmark[i].z<<std::endl;

        // 第二个图
        cv::Point2f pt2_cam = pixel2cam( kps2f_curr_lcam[i], K_lcam );
        cv::Mat pt2_trans = R*( cv::Mat_<double>(3,1) << points_landmark[i].x, points_landmark[i].y, points_landmark[i].z ) + t;
        pt2_trans /= pt2_trans.at<double>(2,0);
        std::cout<<"point in the second camera frame: "<<pt2_cam<<std::endl;
        std::cout<<"point reprojected from second frame: "<<pt2_trans.t()<<std::endl;
        std::cout<<std::endl;
    }

}
void pose_lcam_update(
        const cv::Mat& R,
        const cv::Mat& t)
{

    // update the pose of the left camera //////////////////////////////////////////////////////////////////////////////////////////
    Eigen::Matrix3d rotation_matrix_diff_lcam2imu;
    Eigen::Vector3d translate_vec_diff_lcam2imu;
    cv2eigen(R, rotation_matrix_diff_lcam2imu);
    cv2eigen(t, translate_vec_diff_lcam2imu);
    Eigen::Quaterniond q_diff_lcam2imu = Eigen::Quaterniond(rotation_matrix_diff_lcam2imu);

    // Update the pose with Isometry
    Eigen::Isometry3d pose_diff_lcam = Eigen::Isometry3d::Identity();
    pose_diff_lcam.rotate(rotation_matrix_diff_lcam2imu);
    //todo: check the sign
    pose_diff_lcam.pretranslate(translate_vec_diff_lcam2imu);

    // Update the pose
    pose_lcam = pose_diff_lcam*pose_lcam;
    std::cout << "pose of left camera is " << std::endl << pose_lcam.matrix() << std::endl;

}

void landmark_transform (
        const cv::Mat R,
        const cv::Mat t)
{
    // tranform the landmark point into the frame of the second camera, which will be used as reference frame for landmark points in next iteration
    std::vector<cv::Point3d>::iterator iter_landmark;
    cv::Point3d point_landmark;
    for(iter_landmark = std::begin(points_landmark); iter_landmark != std::end(points_landmark); ++iter_landmark)
    {
        point_landmark = *iter_landmark;
//        std::cout << "Landmark point before transformation is " << point_landmark << std::endl;
        cv::Mat pt_trans = R.inv()*((cv::Mat_<double>(3,1) << point_landmark.x, point_landmark.y, point_landmark.z) - t);
        point_landmark.x = pt_trans.at<double>(0, 0);
        point_landmark.y = pt_trans.at<double>(0, 1);
        point_landmark.z = pt_trans.at<double>(0, 2);
//        std::cout << "Landmark point after transformation is " << point_landmark << std::endl;
        *iter_landmark = point_landmark; //push back to the points_landmark
    }
    std::cout << "3D Points Transformation Finished "<< std::endl;

}

void ba_3d2d_g2o_test (
        const std::vector< cv::Point3d > points_3d,
        const std::vector< cv::KeyPoint >& keypoints_2,
        const std::vector< cv::DMatch >& matches,
        const cv::Mat& R,
        const cv::Mat& t )
{

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block;  // 每个误差项优化变量维度为6,3.
    // 第1步：创建一个线性求解器LinearSolver
//    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>();
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<Block::PoseMatrixType>();
    // 第2步：创建BlockSolver。并用上面定义的线性求解器初始化
    Block* solver_ptr = new Block( std::unique_ptr<Block::LinearSolverType>(linearSolver) );// 矩阵块求解器
    // 第3步：创建总求解器solver。并从GN, LM, DogLeg 中选一个，再用上述块求解器BlockSolver初始化
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<Block>(solver_ptr) );
    // 第4步：创建终极大boss 稀疏优化器（SparseOptimizer）
    g2o::SparseOptimizer optimizer;     // 图模型
    optimizer.setAlgorithm( solver );   // 设置求解器
    optimizer.setVerbose( true );       // 打开调试输出

    // parameter: camera intrinsics
    g2o::CameraParameters* camera = new g2o::CameraParameters (
            K_lcam.at<double> ( 0,0 ), Eigen::Vector2d ( K_lcam.at<double> ( 0,2 ), K_lcam.at<double> ( 1,2 ) ), 0
    );
    camera->setId ( 0 );
    optimizer.addParameter ( camera );

    // vertex, camera pose
    g2o::VertexSE3Expmap* pose_cam = new g2o::VertexSE3Expmap();
    Eigen::Matrix3d R_mat;
    R_mat <<
          R.at<double> ( 0,0 ), R.at<double> ( 0,1 ), R.at<double> ( 0,2 ),
            R.at<double> ( 1,0 ), R.at<double> ( 1,1 ), R.at<double> ( 1,2 ),
            R.at<double> ( 2,0 ), R.at<double> ( 2,1 ), R.at<double> ( 2,2 );

    g2o::SE3Quat pose_cam_initial = g2o::SE3Quat(R_mat, Eigen::Vector3d(t.at<double> ( 0,0 ), t.at<double> ( 1,0 ), t.at<double> ( 2,0 )));
    pose_cam->setEstimate(pose_cam_initial);
    pose_cam->setId(0);
    optimizer.addVertex(pose_cam);

    // vertex, set of landmark_points
    int index = 1;
    for ( const cv::Point3d p:points_3d )
    {
        g2o::VertexSBAPointXYZ* point_landmark = new g2o::VertexSBAPointXYZ();
        point_landmark->setId(index);
        point_landmark->setEstimate(Eigen::Vector3d ( p.x, p.y, p.z ));
        point_landmark->setMarginalized ( true );
        optimizer.addVertex (point_landmark);
        index++;
    }

    // Edge, Reprojection from Landmark Points to Pixel Point
    std::vector<cv::Point2f> points_2f;
    for (int i = 0; i < matches.size(); i++)
    {
        points_2f.push_back(keypoints_2[matches[i].trainIdx].pt);
    }
    index = 1;
    for ( const cv::Point2f p:points_2f )
    {
        g2o::EdgeProjectXYZ2UV* ReprojectionError = new g2o::EdgeProjectXYZ2UV();
        ReprojectionError->setId(index);
        ReprojectionError->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(index)));
        ReprojectionError->setVertex(1, pose_cam);
        ReprojectionError->setMeasurement ( Eigen::Vector2d ( p.x, p.y ) );
        ReprojectionError->setParameterId ( 0,0 );
        ReprojectionError->setInformation ( Eigen::Matrix2d::Identity() );
        optimizer.addEdge ( ReprojectionError );
        index++;
    }

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    optimizer.setVerbose ( true );
    optimizer.initializeOptimization();
    optimizer.optimize ( 100 );
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>> ( t2-t1 );
    std::cout<<"optimization costs time: "<<time_used.count() <<" seconds."<<std::endl;

    std::cout<<std::endl<<"after optimization using g2o test:"<<std::endl;
    std::cout<<"T="<<std::endl<<Eigen::Isometry3d ( pose_cam->estimate() ).matrix() <<std::endl;


}

void ba_3d2d_ceres (
        const std::vector< cv::Point3d > points_3d,
        const std::vector< cv::KeyPoint >& keypoints_2,
        const std::vector< cv::DMatch >& matches,
        const cv::Mat R_initial_cv,
        const cv::Mat t_initial_cv
        )
{
    // Get initial guess ///////////////////////////////////////////////////////////////////////////////////////////

    // initial guess for translation vector
    double t_initial[3];
    t_initial[0] = t_initial_cv.at<double>(0, 0);
    t_initial[1] = t_initial_cv.at<double>(1, 0);
    t_initial[2] = t_initial_cv.at<double>(2, 0);

    // initial guess for rotation matrix
    Eigen::Matrix3d R_initial_eigen;
    cv2eigen(R_initial_cv, R_initial_eigen);
    // convert to quaternion
    double q_initial[4];
    Eigen::Quaterniond q_initial_eigen = Eigen::Quaterniond(R_initial_eigen);
    // coeffs: x, y, z, w.
    // ceres w, x, y, z
    q_initial[0] = q_initial_eigen.coeffs()[3]; //w
    q_initial[1] = q_initial_eigen.coeffs()[0]; //x
    q_initial[2] = q_initial_eigen.coeffs()[1]; //y
    q_initial[3] = q_initial_eigen.coeffs()[2]; //z
    // convert to angle axis, which is used in ceres
    double angleaxis_initial[3];
    ceres::QuaternionToAngleAxis(q_initial, angleaxis_initial);

    // Tranform, angle axis [3] and t[3]
    double T[6];
    T[0] = angleaxis_initial[0];
    T[1] = angleaxis_initial[1];
    T[2] = angleaxis_initial[2];
    T[3] = t_initial[0];
    T[4] = t_initial[1];
    T[5] = t_initial[2];

    int Num_Ele_P_Blk = 3;
//    double Pts[Num_Ele_P_Blk*matches.size()] = {0}; // Length: 3xNum_Pts
//    double* ptr_Pts = &Pts[0];

    // Problem Building /////////////////////////////////////////////////////////////////////////////////////////////////////
    ceres::Problem ba_problem;
    // Output residual: 2D;
    // Input Parameter Block Nr. 1: 6D;
    // Input Parameter Block Nr. 2: 3D;

    // Ordering for marginalization
    // element in group 0 comes with higher prioritiy in marginalization
    ceres::ParameterBlockOrdering* ordering = new ceres::ParameterBlockOrdering;
    ordering->AddElementToGroup(T, 1);

    // Adding Residual Block to the problem

    // Vector of pointers of all landmark points
    std::vector<double*> ptr_Pts;

    // There ought to be one residual block for one observation
    for (int i = 0; i < matches.size(); i++)
    {
        // Observation
        cv::Point2f point_2d = keypoints_2[matches[i].trainIdx].pt;
//        double* ptr_P = ptr_Pts + Num_Ele_P_Blk*pt_index[i];

        // Pointer to landmark point which comes with the initial guess
        // Note：此处一定要使用new来新建数组，因为new会给这个变量重新分配内存，如果不使用new，local_ptr的地址在这个循环中一直保持不变，实际上就会没有给这个优化问题创建新的优化变量
        double* local_Pt = new double[3]{points_3d[i].x, points_3d[i].y, points_3d[i].z};
        // Push back the pointer
        ptr_Pts.push_back(local_Pt);
        // todo: consider using factory construction function
        ceres::CostFunction* ba_cost = new ceres::AutoDiffCostFunction<BA_3D2D_COST, 2, 6, 3>(new BA_3D2D_COST(point_2d.x, point_2d.y));
        // 传递进这个Residual Block的是每个Obseration对应的待优化变量的指针。传递数组本身也就相当于传递了数组的指针。
        ba_problem.AddResidualBlock(ba_cost, nullptr, T, local_Pt);
        // Prioritize landmark points in marginalization
        ordering->AddElementToGroup(local_Pt, 0);
    }

    std::cout << "Problem Building Finished" << std::endl;

    ceres::Solver::Options ba_options;
    ba_options.max_num_iterations = 30;
//    ba_options.linear_solver_type = ceres::DENSE_QR;
    ba_options.linear_solver_type = ceres::SPARSE_SCHUR;
    ba_options.linear_solver_ordering.reset(ordering);
    ba_options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary ba_summary;
    ceres::Solve(ba_options, &ba_problem, &ba_summary);
    std::cout << ba_summary.BriefReport() << "\n";

    // Postprocessing to get the optimized T and Pts //////////////////////////////////////////////////////////////////////////////////////////////
    double angleaxis_opt[3] = {T[0], T[1], T[2]}; // Optimized Angle Axis, consisting of a normal vector, representing the rotation axis and rotation angle
    double translate_opt[3] = {T[3], T[4], T[5]}; // Optimized Translation

    double q_opt[4];

    ceres::AngleAxisToQuaternion(angleaxis_opt, q_opt); // quaternion: w, x, y, z
    Eigen::Quaterniond q_opt_eigen = Eigen::Quaterniond(q_opt[0], q_opt[1], q_opt[2], q_opt[3]); // Oder: w, x, y, z
    q_opt_eigen.normalize();

    // Construct a Rotation Matrix in Eigen
    Eigen::Isometry3d T_eigen = Eigen::Isometry3d::Identity();
    T_eigen.rotate(q_opt_eigen);
    T_eigen.pretranslate(Eigen::Vector3d(translate_opt[0], translate_opt[1], translate_opt[2]));
    std::cout << "camera pose after optimization is " <<  std::endl << T_eigen.matrix() << std::endl;

    // Pointer to the first element in the array of landmark point
    std::cout << "landmark point pointer after optimization is " <<  std::endl << ptr_Pts[0] << std::endl;
    // Pointer to the third element in the array of landmark point, which is the z (depth)
    std::cout << "depth of the landmark point after optimization is " <<  std::endl << *(ptr_Pts[0] + 2) << std::endl;

}
/**
* This tutorial demonstrates simple receipt of messages over the ROS system.
*/


//static void IMU_Callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr)
//{
//
//    pose_qx_imu = imu_msg_ptr->orientation.x;
//    pose_qy_imu = imu_msg_ptr->orientation.y;
//    pose_qz_imu = imu_msg_ptr->orientation.z;
//    pose_qw_imu = imu_msg_ptr->orientation.w;
//
//    tf::Matrix3x3 tmp_display(tf::Quaternion(pose_qx_imu, pose_qy_imu, pose_qz_imu, pose_qw_imu));
//    tmp_display.getRPY(pose_roll_imu, pose_pitch_imu, pose_yaw_imu);
//
////    std::cout << "IMU Roll is " << pose_roll_imu << std::endl;
////    std::cout << "IMU PITCH is " << pose_pitch_imu << std::endl;
////    std::cout << "IMU YAW is " << pose_yaw_imu << std::endl;
//
//    imu_data imu_data_tmp;
//    imu_data_tmp.time = imu_msg_ptr->header.stamp.toSec();
//
//    imu_data_tmp.linear_acceleration_x = imu_msg_ptr->linear_acceleration.x; // m/s^2
//    imu_data_tmp.linear_acceleration_y = imu_msg_ptr->linear_acceleration.y;
//    imu_data_tmp.linear_acceleration_z = imu_msg_ptr->linear_acceleration.z;
//
//    imu_data_tmp.angular_velocity_x = imu_msg_ptr->angular_velocity.x; // rad/sec
//    imu_data_tmp.angular_velocity_y = imu_msg_ptr->angular_velocity.y;
//    imu_data_tmp.angular_velocity_z = imu_msg_ptr->angular_velocity.z;
//
//    imu_data_tmp.orientation_x = imu_msg_ptr->orientation.x;
//    imu_data_tmp.orientation_y = imu_msg_ptr->orientation.y;
//    imu_data_tmp.orientation_z = imu_msg_ptr->orientation.z;
//    imu_data_tmp.orientation_w = imu_msg_ptr->orientation.w;
//
//    imu_data_buffer.push_front(imu_data_tmp);
//    if (imu_data_buffer.size() > 100)
//    {
//        imu_data_buffer.pop_back();
//    }
//    std::cout << "Number of Elements in IMU_DATA_QUEUE is " << imu_data_buffer.size() << std::endl;
//
//    // Update the imu_pose
//
////    pose_x_imu += delta_time_imu*imu_msg_ptr->linear_acceleration.x;
////    pose_y_imu += delta_time_imu*imu_msg_ptr->linear_acceleration.y;
////    pose_z_imu += delta_time_imu*imu_msg_ptr->linear_acceleration.z;
////
////    pose_pitch_imu += delta_time_imu*imu_msg_ptr->angular_velocity.x;
////    pose_pitch_imu += delta_time_imu*imu_msg_ptr->angular_velocity.x;
////    pose_pitch_imu += delta_time_imu*imu_msg_ptr->angular_velocity.x;
//
//    // Get the difference in time stamp
//    // todo:解决时间戳初始化的问题
//    float delta_time_imu = 0.1;
//    std::cout << "time difference between two imu msgs is " << delta_time_imu << std::endl;
//
//    // Using Cosine Direct Matrix (rotation matrix) Update -- According to An introduction to inertial navigation (eq. 35)
//
//    // Get the skew symmetric matrix of rotation difference during one sample period: An introduction to inertial navigation (eq. 37)
//    float B00 = 0.0;
//    float B01 = -(imu_msg_ptr->angular_velocity.z)*delta_time_imu;
//    float B02 = (imu_msg_ptr->angular_velocity.y)*delta_time_imu;
//
//    float B10 = -B01;
//    float B11 = 0.0;
//    float B12 = -(imu_msg_ptr->angular_velocity.x)*delta_time_imu;
//
//    float B20 = -B02;
//    float B21 = -B12;
//    float B22 = 0;
//
//    Eigen::Matrix3f B_matrix;
//    B_matrix << B00, B01, B02, B10, B11, B12, B20, B21, B22;
//
//    // Get the sigma of the rotation: An introduction to inertial navigation (between eq. 37 and Eq. 38)
//    Eigen::Vector3f w_mode;
//    w_mode << imu_msg_ptr->angular_velocity.x*delta_time_imu, imu_msg_ptr->angular_velocity.y*delta_time_imu, imu_msg_ptr->angular_velocity.z*delta_time_imu;
//    float sigma_rotation = w_mode.norm();
//
//    // Update the rotation matrix Eq 41
//    rotation_matrix_imu2imu = rotation_matrix_imu2imu*(Eigen::Matrix3f::Identity() + (sin(sigma_rotation)/sigma_rotation)*B_matrix) + (((1.0 - cos(sigma_rotation))/(sigma_rotation*sigma_rotation))*B_matrix*B_matrix);
//
//    std::cout << "current rotation matrix of imu_odom is " << std::endl << rotation_matrix_imu2imu << std::endl;
//
//    // Using Quaternion to Update --> https://zhuanlan.zhihu.com/p/144954577 Eq5
//    Eigen::Quaterniond q_diff_imu(cos(sigma_rotation/2.0), sin(sigma_rotation/2.0)*w_mode(0)/sigma_rotation, sin(sigma_rotation/2.0)*w_mode(1)/sigma_rotation, sin(sigma_rotation/2.0)*w_mode(2)/sigma_rotation);
//    q_diff_imu.normalize();
//    q_imu2imu = q_diff_imu*q_imu2imu;
//    q_imu2imu.normalize();
//
////    // todo: try update pose with isometry transform
////    // Update the pose with Isometry
////    Eigen::Isometry3d pose_diff_imu = Eigen::Isometry3d::Identity();
////    pose_diff_imu.rotate(q_diff_imu);
////    //todo: check the sign
////    pose_diff_imu.pretranslate(translate_vec_diff_imu2imu);
////
////    pose_imu = pose_diff_imu*pose_imu;
////    std::cout << "pose of imu is " << std::endl << pose_imu.matrix() << std::endl;
////
////    pose_x_imu = 0;
////    pose_y_imu = 0;
////    pose_z_imu = 0;
////
//    // Eigen w, x, y, z
//    pose_qw_imu = q_imu2imu.coeffs()(0);
//    pose_qx_imu = q_imu2imu.coeffs()(1);
//    pose_qy_imu = q_imu2imu.coeffs()(2);
//    pose_qz_imu = q_imu2imu.coeffs()(3);
//
//    std::cout << "current quaternion of imu_odom is " << std::endl << q_imu2imu.coeffs() << std::endl;
//
//    if (!gnss_odom_initialized)
//    // if the gnss_odom is not yet initialized,
//    // no imu_odom should be published (what will happen in the section below);
//    // but the imu_data_buffer should be maintained (what happens in the section above)
//    {
//        std::cout << "no imu_odom should be published" << std::endl;
//        return;
//    }
//
//    // publish the imu_odom msg
//    nav_msgs::Odometry imu_odom;
//    imu_odom.header.stamp = imu_msg_ptr->header.stamp;
//
////    imu_odom.pose.pose.position.x = pose_x_imu;
////    imu_odom.pose.pose.position.y = pose_y_imu;
////    imu_odom.pose.pose.position.z = pose_z_imu;
//    imu_odom.pose.pose.position.x = pose_x_gnss;
//    imu_odom.pose.pose.position.y = pose_y_gnss;
//    imu_odom.pose.pose.position.z = pose_z_gnss;
//
//    imu_odom.pose.pose.orientation.x = pose_qx_imu;
//    imu_odom.pose.pose.orientation.y = pose_qy_imu;
//    imu_odom.pose.pose.orientation.z = pose_qz_imu;
//    imu_odom.pose.pose.orientation.w = pose_qw_imu;
//
//    imu_odom.header.frame_id = "imu_link";
//    imu_odom.child_frame_id = "imu_odom";
//
//    imu_odom_pub.publish(imu_odom);
//
//    static tf::TransformBroadcaster br_imu2imu; // create a broadcaster for transform from imu to imu_odom
//    static tf::Transform tf_imu2imu; // Transform from imu_link to imu_odom
//
//    tf_imu2imu.setOrigin(tf::Vector3(pose_x_imu, pose_y_imu, pose_z_imu)); // Translation between frames
//    tf::Quaternion q_imu2imu_tf = tf::Quaternion (pose_qx_imu, pose_qy_imu, pose_qz_imu, pose_qw_imu);
//
//    q_imu2imu_tf.normalize(); // normalize to enhance numeric stability
//    tf_imu2imu.setRotation(q_imu2imu_tf);
//    br_imu2imu.sendTransform(tf::StampedTransform(tf_imu2imu, imu_msg_ptr->header.stamp, "imu_link", "imu_odom")); // send the transform with time stamp
////    std::cout << "Time Stamp of the imu Message is " << imu_msg_ptr->header.stamp << std::endl;
//
//    stamp_imu_msg = imu_msg_ptr->header.stamp.toSec();
//
//
//}

static void IMU_Callback_Class(const sensor_msgs::ImuConstPtr& imu_msg_ptr)
{
    imu_frame::ptr imu_frame_ptr = imu_frame_ptr->create_frame();
    imu_frame_ptr->update_frame(imu_msg_ptr);
    map_ptr->imu_frames_.push_front(imu_frame_ptr); // 从前端插入
    map_ptr->update_imu_pose(); //在插入最新的imu_frame之后调用
    imu_frame_ptr->pub_odom(imu_odom_pub);
    std::cout << "Number of Elements in IMU_DATA_QUEUE is " << map_ptr->imu_frames_.size() << std::endl;
}

//static void GNSS_Callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr)
//{
//    if (imu_data_buffer.empty()) // If there is no imu data available yet, because the gnss need orientation information from imu data
////    if (map_ptr->imu_frames_.empty()) // If there is no imu data available yet, because the gnss need orientation information from imu data
//    {
//        ROS_INFO("No IMU_DATA found for Initialization");
//        return;
//    }
//    if (!imu_data_buffer.empty() && !gnss_odom_initialized)
////    if (!map_ptr->imu_frames_.empty() && !gnss_odom_initialized)
//        // If there is already imu data available for synchronisation
//        // AND
//        // the odom is not yet initialized with GNSS Data
//    {
//        std::cout << "odom not yet initialized" << std::endl;
////        std::cout << "initial longitude is " << nav_sat_fix_ptr->longitude << std::endl;
////        std::cout << "initial latitude is " << nav_sat_fix_ptr->latitude << std::endl;
////        std::cout << "initial altitude is " << nav_sat_fix_ptr->altitude << std::endl;
//        geo_converter.Reset(nav_sat_fix_ptr->latitude, nav_sat_fix_ptr->longitude, nav_sat_fix_ptr->altitude);// 把此时的gps位置设为原点
//
//        pose_x_gnss = 0;
//        pose_y_gnss = 0;
//        pose_z_gnss = 0;
//
//        //synchronise the orientation with imu data，从imu得到orientation
//        get_orientation(nav_sat_fix_ptr->header.stamp.toSec(), imu_data_buffer);
//
//        // Initialize the imu_pose with current gnss data
//        pose_x_imu = 0;
//        pose_y_imu = 0;
//        pose_z_imu = 0;
//
//        pose_qx_imu = pose_qx_gnss;
//        pose_qy_imu = pose_qy_gnss;
//        pose_qz_imu = pose_qz_gnss;
//        pose_qw_imu = pose_qw_gnss;
//
//        stamp_imu_msg = nav_sat_fix_ptr->header.stamp.toSec();
//        // todo: check the order
//        q_imu2imu = Eigen::Quaterniond(pose_qw_imu, pose_qx_imu, pose_qy_imu, pose_qz_imu); // Oder: w, x, y, z
//        q_imu2imu.normalize();
//        rotation_matrix_imu2imu = q_imu2imu.matrix().cast<float>();
//
//        std::cout << "initial imu rotation matrix is " << std::endl << rotation_matrix_imu2imu << std::endl;
//
//        // Initialize the pose of imu
//        pose_imu.rotate(q_imu2imu);
//        //todo: check the sign
//        pose_imu.pretranslate(Eigen::Vector3d(pose_x_imu, pose_y_imu, pose_z_imu));
//        std::cout << "initial pose of imu" <<  std::endl << pose_imu.matrix() << std::endl;
//
//        // Initialize the lcam_pose with current gnss data
//        pose_x_lcam = 0;
//        pose_y_lcam = 0;
//        pose_z_lcam = 0;
//
//        pose_qx_lcam = pose_qx_gnss;
//        pose_qy_lcam = pose_qy_gnss;
//        pose_qz_lcam = pose_qz_gnss;
//        pose_qw_lcam = pose_qw_gnss;
//
//        stamp_lcam_msg = nav_sat_fix_ptr->header.stamp.toSec();
//        // todo: check the order
//        q_lcam2imu = Eigen::Quaterniond(pose_qw_lcam, pose_qx_lcam, pose_qy_lcam, pose_qz_lcam); // Order: w, x, y, z
//        q_lcam2imu.normalize();
//        std::cout << "initial lcam quaternion is " << std::endl << q_lcam2imu.coeffs() << std::endl;
////        rotation_matrix_lcam2imu = q_lcam2imu.matrix().cast<float>();
//        rotation_matrix_lcam2imu = q_lcam2imu.matrix();
//        std::cout << "initial lcam rotation matrix is " << std::endl << rotation_matrix_lcam2imu << std::endl;
//
//        // Initialize the pose of lcam
//        pose_lcam.rotate(q_lcam2imu);
//        //todo: check the sign
//        pose_lcam.pretranslate(Eigen::Vector3d(pose_x_lcam, pose_y_lcam, pose_z_lcam));
//        std::cout << "initial pose of left camera " <<  std::endl << pose_lcam.matrix() << std::endl;
//
//        // gnss_odom initilization finished
//
//        gnss_odom_initialized = true;
//    }
//    else
//        // If the odom is initialized with GNSS Data
//    {
//        std::cout << "odom already initialized" << std::endl;
//        //update the position with GNSS Data
//        geo_converter.Forward(nav_sat_fix_ptr->latitude, nav_sat_fix_ptr->longitude, nav_sat_fix_ptr->altitude, pose_x_gnss, pose_y_gnss, pose_z_gnss);
//        //synchronise the orientation with imu data
//        get_orientation(nav_sat_fix_ptr->header.stamp.toSec(), imu_data_buffer);
//    }
//
//    // calculate the euler angles of the gnss_odom
//    tf::Matrix3x3 tmp(tf::Quaternion(pose_qx_gnss, pose_qy_gnss, pose_qz_gnss, pose_qw_gnss));
//    tmp.getRPY(pose_roll_gnss, pose_pitch_gnss, pose_yaw_gnss);
//
//    // expand the gnss_data deque
//    gnss_data gnss_data_tmp;
//    gnss_data_tmp.time = nav_sat_fix_ptr->header.stamp.toSec();
//    gnss_data_tmp.latitude = nav_sat_fix_ptr->latitude;
//    gnss_data_tmp.longitude = nav_sat_fix_ptr->longitude;
//    gnss_data_tmp.altitude = nav_sat_fix_ptr->altitude;
//    gnss_data_tmp.status = nav_sat_fix_ptr->status.status;
//    gnss_data_tmp.service = nav_sat_fix_ptr->status.service;
//    gnss_data_tmp.local_E = pose_x_gnss;
//    gnss_data_tmp.local_N = pose_y_gnss;
//    gnss_data_tmp.local_U = pose_z_gnss;
//
//    gnss_data_buffer.push_front(gnss_data_tmp);
//    if (gnss_data_buffer.size() > 100)
//    {
//        gnss_data_buffer.pop_back();
//    }
//    std::cout << "Number of Elements in GNSS_DATA_QUEUE is " << gnss_data_buffer.size() << std::endl;
//
////    std::cout << "loacl x is " << pose_x_gnss << std::endl;
////    std::cout << "loacl y is " << pose_y_gnss << std::endl;
////    std::cout << "loacl z is " << pose_z_gnss << std::endl;
//
//// publisher the gnss_odom msg
//    nav_msgs::Odometry gnss_odom;
//    gnss_odom.header.stamp = nav_sat_fix_ptr->header.stamp;
//
//    gnss_odom.pose.pose.position.x = pose_x_gnss;
//    gnss_odom.pose.pose.position.y = pose_y_gnss;
//    gnss_odom.pose.pose.position.z = pose_z_gnss;
//
//    gnss_odom.pose.pose.orientation.x = pose_qx_gnss;
//    gnss_odom.pose.pose.orientation.y = pose_qy_gnss;
//    gnss_odom.pose.pose.orientation.z = pose_qz_gnss;
//    gnss_odom.pose.pose.orientation.w = pose_qw_gnss;
//
//    gnss_odom.header.frame_id = "imu_link";
//    gnss_odom.child_frame_id = "gnss_odom";
//
//    gnss_odom_pub.publish(gnss_odom);
//
//    static tf::TransformBroadcaster br_imu2gnss; // create a broadcaster for transform from imu to gnss_odom
//
//    static tf::Transform tf_imu2gnss; // Transform from imu_link to gnss_odom
//    tf_imu2gnss.setOrigin(tf::Vector3(pose_x_gnss, pose_y_gnss, pose_z_gnss)); // Translation between frames
//    tf::Quaternion q_imu2gnss = tf::Quaternion (pose_qx_gnss, pose_qy_gnss, pose_qz_gnss, pose_qw_gnss);
////    tf::Quaternion q_imu2gnss = tf::Quaternion (0, 0, 0, 1); // Rotation between frames
//    q_imu2gnss.normalize(); // normalize to enhance numeric stability
//    tf_imu2gnss.setRotation(q_imu2gnss);
//
////    br_imu2gnss.sendTransform(tf::StampedTransform(tf_imu2gnss, ros::Time::now(), "imu_link", "gnss_odom")); // send the transform with time stamp
//    br_imu2gnss.sendTransform(tf::StampedTransform(tf_imu2gnss, nav_sat_fix_ptr->header.stamp, "imu_link", "gnss_odom"));
//    std::cout << "Time Stamp of the GNSS Message is " << nav_sat_fix_ptr->header.stamp << std::endl;
//
//}

static void GNSS_Callback_Class(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr)
{
    if (map_ptr->imu_frames_.empty()) // If there is no imu data available yet, because the gnss need orientation information from imu data
    {
        ROS_INFO("No IMU_DATA found for Initialization");
        return;
    }
    if (!map_ptr->imu_frames_.empty() && !gnss_odom_initialized)
        // If there is already imu data available for synchronisation
        // AND
        // the odom is not yet initialized with GNSS Data
    {
        std::cout << "initialization of gnss odom" << std::endl;

        gnss_frame::ptr gnss_frame_ptr = gnss_frame_ptr->create_frame();
        gnss_frame_ptr->set_origin(nav_sat_fix_ptr, map_ptr);
        map_ptr->add_keygnssframe(gnss_frame_ptr);
        gnss_frame_ptr->pub_odom(gnss_odom_pub);

        // Initialize the imu_pose to origin using gnss frame
        map_ptr->set_imu_origin(gnss_frame_ptr);


        stamp_lcam_msg = nav_sat_fix_ptr->header.stamp.toSec();

        pose_lcam = gnss_frame_ptr->T_cw_;
        std::cout << "initial pose of left camera " <<  std::endl << pose_lcam.matrix() << std::endl;

        gnss_odom_initialized = true;
    }
    else
        // If the odom is initialized with GNSS Data
    {
        std::cout << "odom already initialized" << std::endl;
        //update the position with GNSS Data

        gnss_frame::ptr gnss_frame_ptr = gnss_frame_ptr->create_frame();
        gnss_frame_ptr->set_frame(nav_sat_fix_ptr, map_ptr);
        map_ptr->add_keygnssframe(gnss_frame_ptr);
        // publisher the gnss_odom msg
        gnss_frame_ptr->pub_odom(gnss_odom_pub);
    }

    // calculate the euler angles of the gnss_odom
    tf::Matrix3x3 tmp(tf::Quaternion(pose_qx_gnss, pose_qy_gnss, pose_qz_gnss, pose_qw_gnss));
    tmp.getRPY(pose_roll_gnss, pose_pitch_gnss, pose_yaw_gnss);



}

//static void LCAM_Callback(const sensor_msgs::ImageConstPtr& lcam_img_msg_ptr)
//{
//
//    count_lcam_callback += 1;
//    cv_bridge::CvImagePtr lcam_cv_ptr = cv_bridge::toCvCopy(lcam_img_msg_ptr);
//    ROS_INFO("Get a image from left camera");
//    std::cout << "Number of column is " << lcam_cv_ptr->image.cols << std::endl;
//    curr_lcam_img = lcam_cv_ptr->image;
//    std::cout << "the type of the read image is " << curr_lcam_img.type() << std::endl;
//
//    // Maintain the lcam_data_buffer//////////////////////////////////////////////////////////////////////////////
//    lcam_data lcam_data_tmp;
//    lcam_data_tmp.lcam_data_ptr = lcam_cv_ptr;
//    lcam_data_buffer.push_front(lcam_data_tmp);
//
//    if (lcam_data_buffer.size() > 10)
//    {
//        lcam_data_buffer.pop_back();
//    }
//    std::cout << "Number of Elements in LCAM_DATA_QUEUE is " << lcam_data_buffer.size() << std::endl;
//
//
//    if (!gnss_odom_initialized)
//        // if the gnss_odom is not yet initialized,
//        // no lcam_odom should be published (what will happen in the section below);
//        // but the lcam_data_buffer should be maintained (what happens in the section above)
//    {
//        std::cout << "no lcam_odom should be published" << std::endl;
//        return;
//    }
//
//    // Check if current image is the first image after gnss initialization
//    if (pre_lcam_img.empty())
//    {
//        std::cout << "First image after gnss initialization" << std::endl;
//
//        // Update the previous image with current image
//        pre_lcam_img = curr_lcam_img.clone();
//        return;
//    }
//
//    std::cout << "First element of the previous image is " << pre_lcam_img.at<cv::Vec3b>(100, 100) << std::endl;
//    std::cout << "First element of the current image is " << curr_lcam_img.at<cv::Vec3b>(100, 100) << std::endl;
//
//    if(first_match)
//    {
//        std::cout<<"It is the first matching!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1"<<std::endl;
//
//        // 2D-2D Feature Matching
//        find_feature_matches (pre_lcam_img, curr_lcam_img);
//        std::cout<<"一共找到了"<<keypoints_pre_lcam.size() <<"组匹配点"<<std::endl;
//        std::cout<<"Size of the keypoints, 2f, landmark points, descriptor are " << keypoints_pre_lcam.size() <<" "<< kps2f_pre_lcam.size() << " " << points_landmark.size() << " " << descptor_pre_lcam.rows << std::endl;
////        match_draw_store();
//        //-- 估计两张图像间运动
//        cv::Mat R, t;
//        pose_estimation_2d2d(R, t);
//
//        cv::Mat pose_lcam_pre_cv;
//        eigen2cv(pose_lcam.matrix(), pose_lcam_pre_cv);
//        pose_lcam_update(R, t);
//        cv::Mat pose_lcam_curr_cv;
//        eigen2cv(pose_lcam.matrix(), pose_lcam_curr_cv);
//
//        //-- 三角化
//        // the resulted landmark points are in the frame of first camera
////        triangulation_2f(pose_lcam_pre_cv, pose_lcam_curr_cv);
//        triangulation_2f_rel(R, t);
//        std::cout << "first triangulated feature points are "<< std::endl << points_landmark[0]
//                  << std::endl << points_landmark[10]
//                  << std::endl << points_landmark[30] << std::endl;
//
////        //-- 验证三角化点与特征点的重投影关系
////        triangulation_verify(R, t);
//
//        // 3d-2d Feature Matching through pnp: 3d point in frame of first camera, 2d feature in second camera ////////////////////////////////////
//        // Points_landmark should be represented in the frame of the first camera
//        cv::Mat R_pnp, t_pnp;
//        pose_estimation_3d2d_2f(R_pnp, t_pnp);
//        std::cout << "the type of the R_pnp is " << R_pnp.type() << std::endl;
//        rotationmatrixToEulerAngles(R_pnp);
//
//        std::cout<<"R_pnp="<<std::endl<<R_pnp<<std::endl;
//        std::cout<<"t_pnp="<<std::endl<<t_pnp<<std::endl;
//
////        ba_2d2d_direct_ceres(R_pnp, t_pnp);
//
////        // 3d-2d Feature Matching through bundle adjustment: 3d point in frame of first camera, 2d feature in second camera ////////////////////////////////////
////
////        cv::Mat R_ba, t_ba;
////        std::vector<cv::Point3d> points_landmark_ba;
////        ba_3d2d_g2o(points_landmark, kps2f_curr_lcam, R_pnp, t_pnp, R_ba, t_ba, points_landmark_ba);
////        R_pnp = R_ba;
////        t_pnp = t_ba;
////        points_landmark.swap(points_landmark_ba);
////        std::cout<<"R_ba="<<std::endl<<R_pnp<<std::endl;
////        std::cout<<"t_ba="<<std::endl<<t_pnp<<std::endl;
//
////    ba_3d2d_ceres(points_landmark, keypoints_curr_lcam, matches, R_pnp, t_pnp);
//
//        //Updating the landmark point and keypoint //////////////////////////////////////////////////////////////////////////////////
//
//        // Transform the triangulated landmark point to the frame of the current camera
//        landmark_transform(R_pnp, t_pnp);
//
//        // Update the keypoint (2f) and descriptor of the previous image keypoints_pre_lcam with key point of current image
//        kps2f_pre_lcam.swap(kps2f_curr_lcam);
//        keypoints_pre_lcam.swap(keypoints_curr_lcam);
//        descptor_pre_lcam = descptor_curr_lcam.clone();
//        std::cout<<"Size of the keypoints, 2f, landmark points are " << keypoints_pre_lcam.size() <<" "<< kps2f_pre_lcam.size() << " " << points_landmark.size() << std::endl;
//        // Update the previous image with current image
//        pre_lcam_img = curr_lcam_img.clone();
//        first_match = false;
//        return;
//
//    }
//
//    // Track the keypoint with optical flow ////////////////////////////////////////////////////////////////////////////////////////////
//    std::cout << "Here comes a new image for 3d-2d matching!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
//    // Calculate the optical flow and get the tracked keypoint
//
//    std::vector<unsigned char> status;
//    std::vector<float> error;
//    std::vector<cv::Point2f> kps2f_curr_lcam_tmp;
//    cv::calcOpticalFlowPyrLK(pre_lcam_img, curr_lcam_img, kps2f_pre_lcam, kps2f_curr_lcam_tmp, status, error);
//    kps2f_curr_lcam.swap(kps2f_curr_lcam_tmp);
//    std::cout<<"Size of the keypoints, 2f, landmark points are before 3d-2d matching " << keypoints_pre_lcam.size() <<" "<< kps2f_pre_lcam.size() << " " << points_landmark.size() << std::endl;
//
////    // Erase the bad key point, descriptor and landmark points
//    env_update(status);
//
//    // 画出 keypoints
//    cv::Mat img_show = curr_lcam_img.clone();
//    for ( auto kp:kps2f_curr_lcam )
//        cv::circle(img_show, kp, 10, cv::Scalar(0, 240, 0), 1);
//    img_store(img_show, 1);
//
//    // 3d-2d Feature Matching through pnp: 3d point in frame of first camera, 2d feature in second camera ////////////////////////////////////
//    cv::Mat R_pnp, t_pnp;
//    pose_estimation_3d2d_2f(R_pnp, t_pnp);
//    std::cout << "3d-2d Feature Matching Finished "<< std::endl;
//    std::cout<<"R_pnp="<<std::endl<<R_pnp<<std::endl;
//    std::cout<<"t_pnp="<<std::endl<<t_pnp<<std::endl;
//    std::cout<<"mode of t is "<<std::endl<<cv::norm(t_pnp)<<std::endl;
//
//    rotationmatrixToEulerAngles(R_pnp);
//
//    // Update the pose ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    cv::Mat pose_lcam_pre_cv;
//    eigen2cv(pose_lcam.matrix(), pose_lcam_pre_cv);
//    pose_lcam_update(R_pnp, t_pnp);
//    cv::Mat pose_lcam_curr_cv;
//    eigen2cv(pose_lcam.matrix(), pose_lcam_curr_cv);
//
////    //Add new key points and landmark points////////////////////////////////////////////////////////////////////////////////
////    brandnew_kp_add();
////    std::cout<<"Size of the keypoints, 2f, landmark points are " << keypoints_pre_lcam.size() <<" "<< kps2f_pre_lcam.size() << " " << points_landmark.size() << std::endl;
//
//    //triangulation
//    // resulted points_landmark come in the frame of the first camera
////    triangulation_2f(pose_lcam_pre_cv, pose_lcam_curr_cv);
////    triangulation_2f_abs(R_pnp, t_pnp);
//    triangulation_2f_rel(R_pnp, t_pnp);
//
////    //-- 验证三角化点与特征点的重投影关系
////    triangulation_verify(R_pnp, t_pnp);
//
////    // Refinement using ba
////
////    cv::Mat R_ba, t_ba;
////    std::vector< cv::Point3d > points_landmark_ba;
////    ba_3d2d_g2o(points_landmark, kps2f_curr_lcam, R_pnp, t_pnp, R_ba, t_ba, points_landmark_ba);
////    R_pnp = R_ba;
////    t_pnp = t_ba;
////    points_landmark.swap(points_landmark_ba);
////    std::cout<<"R_ba="<<std::endl<<R_pnp<<std::endl;
////    std::cout<<"t_ba="<<std::endl<<t_pnp<<std::endl;
//
//    //Updating the landmark point and keypoint //////////////////////////////////////////////////////////////////////////////////
//
//    // Transform the triangulated landmark point to the frame of the current camera
//    landmark_transform(R_pnp, t_pnp);
//
//    // Update the keypoint (2f) of the previous image keypoints_pre_lcam with key point of current image
//    kps2f_pre_lcam.swap(kps2f_curr_lcam);
//    keypoints_pre_lcam.swap(keypoints_curr_lcam);
//    descptor_pre_lcam = descptor_curr_lcam.clone();
//    // Update the previous image with current image
//    pre_lcam_img = curr_lcam_img.clone();
//
//    std::cout<<"Size of the keypoints, 2f, landmark points are " << keypoints_pre_lcam.size() <<" "<< kps2f_pre_lcam.size() << " " << points_landmark.size() << std::endl;
//
//    // update the position relative to first frame
//    pose_x_lcam = pose_lcam.matrix()(0, 3);
//    pose_y_lcam = pose_lcam.matrix()(1, 3);
//    pose_z_lcam = pose_lcam.matrix()(2, 3);
//
//    // update the rotation matrix relative to first frame
//    rotation_matrix_lcam2imu(0, 0) = pose_lcam.matrix()(0, 0);
//    rotation_matrix_lcam2imu(0, 1) = pose_lcam.matrix()(0, 1);
//    rotation_matrix_lcam2imu(0, 2) = pose_lcam.matrix()(0, 2);
//    rotation_matrix_lcam2imu(1, 0) = pose_lcam.matrix()(1, 0);
//    rotation_matrix_lcam2imu(1, 1) = pose_lcam.matrix()(1, 1);
//    rotation_matrix_lcam2imu(1, 2) = pose_lcam.matrix()(1, 2);
//    rotation_matrix_lcam2imu(2, 0) = pose_lcam.matrix()(2, 0);
//    rotation_matrix_lcam2imu(2, 1) = pose_lcam.matrix()(2, 1);
//    rotation_matrix_lcam2imu(2, 2) = pose_lcam.matrix()(2, 2);
//
//    // Update the Quaternion
//    q_lcam2imu = Eigen::Quaterniond(rotation_matrix_lcam2imu);
//    pose_qx_lcam = q_lcam2imu.coeffs()(0);
//    pose_qy_lcam = q_lcam2imu.coeffs()(1);
//    pose_qz_lcam = q_lcam2imu.coeffs()(2);
//    pose_qw_lcam = q_lcam2imu.coeffs()(3);
//
//    //todo: create the map class to manage the key frame
//
//    // publish the left camera odometry /////////////////////////////////////////////////////////////////////////////////////////
//    nav_msgs::Odometry curr_lcam_odom;
//    curr_lcam_odom.header.stamp = lcam_img_msg_ptr->header.stamp;
//    curr_lcam_odom.header.frame_id = "imu_link";
//    curr_lcam_odom.child_frame_id = "lcam_odom";
//
//    curr_lcam_odom.pose.pose.position.x = pose_x_lcam;
//    curr_lcam_odom.pose.pose.position.y = pose_y_lcam;
//    curr_lcam_odom.pose.pose.position.z = pose_z_lcam;
//
//    curr_lcam_odom.pose.pose.orientation.x = pose_qx_lcam;
//    curr_lcam_odom.pose.pose.orientation.y = pose_qy_lcam;
//    curr_lcam_odom.pose.pose.orientation.z = pose_qz_lcam;
//    curr_lcam_odom.pose.pose.orientation.w = pose_qw_lcam;
//
//    lcam_odom_pub.publish(curr_lcam_odom);
//}

static void LCAM_Callback_Class(const sensor_msgs::ImageConstPtr& lcam_img_msg_ptr)
{

    count_lcam_callback += 1;
    cv_bridge::CvImagePtr lcam_cv_ptr = cv_bridge::toCvCopy(lcam_img_msg_ptr);
    ROS_INFO("Get a image from left camera");
    std::cout << "Number of column is " << lcam_cv_ptr->image.cols << std::endl;
    curr_lcam_img = lcam_cv_ptr->image;
    std::cout << "the type of the read image is " << curr_lcam_img.type() << std::endl;
    frame::ptr frame_ptr = frame_ptr->create_frame();
    std::cout << "id of the color frame ptr is " << frame_ptr->id_ << std::endl;
    frame_ptr->image_ = curr_lcam_img;
    map_ptr->add_keyframe(frame_ptr);


//    // For Tracking ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    std::vector<object_det::ptr> new_objects;
//    map_ptr->curr_frame_->detect_yolo_dark(new_objects);
////    map_ptr->curr_frame_->detect_yolo_trt(new_objects);
//    if (count_lcam_callback == 1)
//    {
//        tracker_2d.initialize(new_objects);
//    }
//    else
//    {
//        tracker_2d.update(new_objects);
//    }
//    tracker_2d.draw_tracks(map_ptr->curr_frame_);


    // Maintain the lcam_data_buffer//////////////////////////////////////////////////////////////////////////////
    lcam_data lcam_data_tmp;
    lcam_data_tmp.lcam_data_ptr = lcam_cv_ptr;
    lcam_data_buffer.push_front(lcam_data_tmp);

    if (lcam_data_buffer.size() > 10)
    {
        lcam_data_buffer.pop_back();
    }
    std::cout << "Number of Elements in LCAM_DATA_QUEUE is " << lcam_data_buffer.size() << std::endl;
    std::cout << "reach line 2066 " << std::endl;
    if (!gnss_odom_initialized)
        // if the gnss_odom is not yet initialized,
        // no lcam_odom should be published (what will happen in the section below);
        // but the lcam_data_buffer should be maintained (what happens in the section above)
    {
        std::cout << "no lcam_odom should be published" << std::endl;
        return;
    }
    std::cout << "reach line 2075 " << std::endl;
    // Check if current image is the first image after gnss initialization
    if (gnss_odom_initialized && map_ptr->id_gnss_ini_ == 0) // 当GNSS已经初始化而且还未进行第一次特征匹配
    {
        std::cout << "First image after gnss initialization" << std::endl;
        frame_ptr->T_cw_ = pose_lcam; // 设定整个map的初始位姿，来自于gnss
        map_ptr->id_gnss_ini_ = frame_ptr->id_;
        return;
    }

    if(!first_match_finished && map_ptr->id_gnss_ini_ != 0) // 当还未进行第一次特征匹配,但是map已经初始化位姿
    {
        std::cout<<"It is the first matching!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1"<<std::endl;

        // 2D-2D Feature Matching
        map_ptr->feature_matches_2d2d();
        // keypoints, kps2f, descriptors updated for pre and curr frame
        std::cout << "size of keypoint in pre and curr frame are " << map_ptr->pre_frame_->keypoints_.size() << " " << map_ptr->curr_frame_->keypoints_.size() << std::endl;
        std::cout << "size of kps2f in pre and curr frame are " << map_ptr->pre_frame_->kps2f_.size() << " " << map_ptr->curr_frame_->kps2f_.size() << std::endl;
        std::cout << "size of descriptors in pre and curr frame are " << map_ptr->pre_frame_->descriptors_.rows << " " << map_ptr->curr_frame_->descriptors_.rows << std::endl;

        //-- 估计两张图像间运动
        cv::Mat R, t;
        map_ptr->pose_estimation_2d2d(map_ptr->pre_frame_, map_ptr->curr_frame_, R, t);

        //-- 三角化
        map_ptr->triangulation(R, t);
        // points_landmark updated for pre and (transformed into) curr frame

        // 3d-2d Feature Matching through pnp: 3d point in frame of first camera, 2d feature in second camera ////////////////////////////////////
        // Points_landmark should be represented in the frame of the first camera
        cv::Mat R_pnp, t_pnp;
        map_ptr->pose_estimation_pnp(map_ptr->pre_frame_, map_ptr->curr_frame_, R_pnp, t_pnp);
        // nothing updated

        map_ptr->show_eulerangles(R_pnp);

        map_ptr->update_pose(R_pnp, t_pnp);
        std::cout << "size of keypoint in pre and curr frame are " << map_ptr->pre_frame_->keypoints_.size() << " " << map_ptr->curr_frame_->keypoints_.size() << std::endl;
        std::cout << "size of kps2f in pre and curr frame are " << map_ptr->pre_frame_->kps2f_.size() << " " << map_ptr->curr_frame_->kps2f_.size() << std::endl;
        std::cout << "size of descriptors in pre and curr frame are " << map_ptr->pre_frame_->descriptors_.rows << " " << map_ptr->curr_frame_->descriptors_.rows << std::endl;

        map_ptr->update_lastpair();

        std::cout << "size of keypoint in pre and curr frame are " << map_ptr->pre_frame_->keypoints_.size() << " " << map_ptr->curr_frame_->keypoints_.size() << std::endl;
        std::cout << "size of kps2f in pre and curr frame are " << map_ptr->pre_frame_->kps2f_.size() << " " << map_ptr->curr_frame_->kps2f_.size() << std::endl;
        std::cout << "size of descriptors in pre and curr frame are " << map_ptr->pre_frame_->descriptors_.rows << " " << map_ptr->curr_frame_->descriptors_.rows << std::endl;

        map_ptr->curr_frame_->pub_odom(lcam_odom_pub);

        first_match_finished = true;
        return;
    }

    // Track the keypoint with optical flow ////////////////////////////////////////////////////////////////////////////////////////////
    std::cout << "Here comes a new image for 3d-2d matching!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    // Calculate the optical flow and get the tracked keypoint
    std::vector<unsigned char> status;
    map_ptr->lk_track(status);
    // keypoints, kps2f, descriptors and landmark points updated for curr frame

    map_ptr->crop_pairframe(status);

    // Draw the tracked keypoints and store the image of the current image
    map_ptr->curr_frame_->draw_and_store_tracking();

    // 3d-2d Feature Matching through pnp: 3d point in frame of first camera, 2d feature in second camera ////////////////////////////////////
    // Points_landmark should be represented in the frame of the first camera
    cv::Mat R_pnp, t_pnp;
    map_ptr->pose_estimation_pnp(map_ptr->pre_frame_, map_ptr->curr_frame_, R_pnp, t_pnp);
    // nothing updated

    map_ptr->show_eulerangles(R_pnp);

    // Update the pose ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    map_ptr->update_pose(R_pnp, t_pnp);

//    //Add new key points and landmark points////////////////////////////////////////////////////////////////////////////////
//    brandnew_kp_add();
//    std::cout<<"Size of the keypoints, 2f, landmark points are " << keypoints_pre_lcam.size() <<" "<< kps2f_pre_lcam.size() << " " << points_landmark.size() << std::endl;

    //-- 三角化
    map_ptr->triangulation(R_pnp, t_pnp);
    // points_landmark updated for pre and (transformed into) curr frame

    map_ptr->update_lastframe();

    map_ptr->curr_frame_->pub_odom(lcam_odom_pub);
}


//static void PointCloud_Callback_al(const sensor_msgs::PointCloud2 curr_scan_msg)
//{
//    if (!gnss_odom_initialized)
//    // If the gnss_odom is not yet already initialized
//    {
//        std::cout << "PointCloud_Callback: gnss_odom is not yet initialized!" << std::endl;
//        return;
//    }
//    ROS_INFO("PointCloudCallback Started");
//    //Get current scan
//    pcl::fromROSMsg(curr_scan_msg, *curr_scan_cloud_ptr); //Convert point cloud msg to point cloud pointer for incoming laser scan
//
//
//
////    // Using TF unsuccessful
////    static tf::TransformListener li_imu2gnss_atlidar;
////    static tf::StampedTransform tf_imu2gnss_atlidar;
////    if (gnss_data_buffer.size() > 5)
////    {
////        li_imu2gnss_atlidar.waitForTransform("/imu_link", "/gnss_odom", curr_scan_msg.header.stamp, ros::Duration(300000.0));
////        li_imu2gnss_atlidar.lookupTransform("/imu_link", "/gnss_odom", curr_scan_msg.header.stamp, tf_imu2gnss_atlidar);
////    }
//
//    // Check if current scan is the first scan
//    if (pre_scan_cloud_ptr->width == 0)
//    {
//        std::cout << "Width of the previous scan is " << pre_scan_cloud_ptr->width << std::endl;
//        pcl::copyPointCloud(*curr_scan_cloud_ptr, *pre_scan_cloud_ptr);// update the previous point cloud by current point cloud;
//
//        // Get the pose of the vehicle under the frame of gnss_odom at this moment
//        // Using the pose from gnss directly
//
//
//        pose_x_lidar = pose_x_gnss;
//        pose_y_lidar = pose_y_gnss;
//        pose_z_lidar = pose_z_gnss;
//
//        pose_pitch_lidar = pose_pitch_gnss;
//        pose_yaw_lidar = pose_yaw_gnss;
//        pose_roll_lidar = pose_roll_gnss;
//
//        Eigen::Translation3f init_translation(pose_x_gnss, pose_y_gnss, pose_z_gnss);
//        Eigen::AngleAxisf init_rotation_x(pose_roll_gnss, Eigen::Vector3f::UnitX());
//        Eigen::AngleAxisf init_rotation_y(pose_pitch_gnss, Eigen::Vector3f::UnitY());
//        Eigen::AngleAxisf init_rotation_z(pose_yaw_gnss, Eigen::Vector3f::UnitZ());
//        pose_lidar = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();
//
//        pose_lidar_pre = pose_lidar;
//
//        return;
//
//    }
//
//    //filter the incoming point cloud message
//    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
//    approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
//    approximate_voxel_filter.setInputCloud(curr_scan_cloud_ptr);
//    approximate_voxel_filter.filter(*curr_scan_cloud_ptr);
//    std::cout << "Filtered cloud contains " << curr_scan_cloud_ptr->size() << std::endl;
//
//    // expand the queue with the point cloud
//    point_cloud point_cloud_tmp;
//
//    point_cloud_tmp.point_cloud_ptr = curr_scan_cloud_ptr;
//    point_cloud_tmp.time = curr_scan_msg.header.stamp.toSec();
//    point_cloud_ptr_buffer.push_front(point_cloud_tmp);
//    if (point_cloud_ptr_buffer.size() > 5)
//    {
//        point_cloud_ptr_buffer.pop_back();
//    }
//    std::cout << "Number of Elements in POINT_CLOUD_PTR_QUEUE is " << point_cloud_ptr_buffer.size() << std::endl;
//
//    ROS_INFO("Scan Msg Converted");
//    std::cout << "Size of the previous Scan is " << pre_scan_cloud_ptr->width << std::endl;
//    std::cout << "Size of the current Scan is " << curr_scan_cloud_ptr->width << std::endl;
//
//
//
//    //Get transform between two frames
//
//    pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
//    ndt_omp->setInputSource(curr_scan_cloud_ptr);
//    ndt_omp->setInputTarget(pre_scan_cloud_ptr);
//    ndt_omp->setTransformationEpsilon (epsilon);
//    ndt_omp->setResolution (res);
//    ndt_omp->setStepSize (step);
//    ndt_omp->setMaximumIterations(100);
//
//    ndt_omp->setNumThreads(NumThreads);
//    ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
//    // get the transform between two poses as guess
//    Eigen::Matrix4f pose_diff = pose_lidar*(pose_lidar_pre.inverse());
//    ROS_INFO("Preparation for Scan Registration Finished");
//    ROS_INFO("Timer Started");
//    auto t1 = ros::WallTime::now();
//    ndt_omp->align(*aligned_scan_cloud_ptr, pose_diff);
//    auto t2 = ros::WallTime::now();
//    ROS_INFO("Timer Ended");
//    std::cout << "Duration : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;
//
//    std::cout << "Normal Distributions Transform has converged after " << ndt_omp->getFinalNumIteration() << " iters" << std::endl;
//    std::cout << "The score is " << ndt_omp->getFitnessScore() << "\n" << std::endl;
//
//    Eigen::Matrix4f Transf = Eigen::Matrix4f::Identity();
//    Transf = ndt_omp->getFinalTransformation();
//    std::cout << "Final Transformation Matrix of the Scan Registation is " << std::endl << Transf << std::endl;
//    cv::Mat t_pcl = (cv::Mat_<double>(3,1) << Transf(0, 3), Transf(1, 3), Transf(2, 3));
//    std::cout << "Mode of the translation is " << std::endl << cv::norm(t_pcl) << std::endl;
//    // todo: transform the point cloud to the current frame of lidar
//
//    // Using Normal NDT
//
////    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
////    ndt.setTransformationEpsilon(0.01);
////    ndt.setStepSize(0.1);
////    ndt.setResolution(1.0);
////    ndt.setMaximumIterations(35);
////    ndt.setInputCloud(curr_scan_cloud_ptr);
////    ndt.setInputTarget(pre_scan_cloud_ptr);
////    auto t1 = ros::WallTime::now();
////    ndt.align(*aligned_scan_cloud_ptr);
////    auto t2 = ros::WallTime::now();
////    std::cout << "Duration : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;
////
//////    std::cout << "Normal Distributions Transform has converged after " << ndt.getFinalNumIteration() << " iters" << std::endl;
////    std::cout << "The score is " << ndt.getFitnessScore() << "\n" << std::endl;
////
////    Eigen::Matrix4f Transf = Eigen::Matrix4f::Identity();
////    Transf = ndt.getFinalTransformation();
////    std::cout << "Final Transformation Matrix of the Scan Registration is " << Transf << std::endl;
//
//
//    //Get current vehicle pose
//
////    // Using Eigen
////    // get the position difference between two frames
////    double delta_pose_x = Transf(0, 3);
////    double delta_pose_y = Transf(1, 3);
////    double delta_pose_z = Transf(2, 3);
////
////    // Update the current position with delta
////    pose_x_lidar += delta_pose_x;
////    pose_y_lidar += delta_pose_y;
////    pose_z_lidar += delta_pose_z;
////
////    // Get the rotation matrix between two frames
////    Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Identity();
////    rotation_matrix = Transf.block(0, 0, 3, 3);
////
////    // Convert the ratation matrix to quaternion
////    Eigen::Quaternionf curr_q_lidar = Eigen::Quaternionf(rotation_matrix);
////    Eigen::Vector4f curr_quat_vector = curr_q_lidar.coeffs(); // in order of x y z w
////
////    // Convert quaternion to euler angles for rotation
////    Eigen::Vector3f euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // YPR
////    std::cout<<"yaw pitch roll = "<<euler_angles.transpose()<<std::endl;
////
////    double delta_pose_yaw = euler_angles(0);
////    double delta_pose_pitch = euler_angles(1);
////    double delta_pose_roll = euler_angles(2);
////
////    // Update the euler angles for pose
////    pose_yaw_lidar += delta_pose_yaw;
////    pose_pitch_lidar += delta_pose_pitch;
////    pose_roll_lidar += delta_pose_roll;
////
////    // Convert the updated pose to quaternion
////    curr_q_lidar = Eigen::AngleAxisf(pose_yaw_lidar, Eigen::Vector3f::UnitX())
////                   *Eigen::AngleAxisf(pose_pitch_lidar, Eigen::Vector3f::UnitY())
////                   *Eigen::AngleAxisf(pose_yaw_lidar, Eigen::Vector3f::UnitZ());
////
////    // Get the coefficient of quaternion
////    curr_quat_vector = curr_q_lidar.coeffs();
////
////    pose_qx_lidar = curr_quat_vector(0);
////    pose_qy_lidar = curr_quat_vector(1);
////    pose_qz_lidar = curr_quat_vector(2);
////    pose_qw_lidar = curr_quat_vector(3);
////
////    // publish the lidar odometry
////    nav_msgs::Odometry curr_lidar_odom;
////    curr_lidar_odom.header.stamp = curr_scan_msg.header.stamp;
////    curr_lidar_odom.header.frame_id = "imu_link";
////    curr_lidar_odom.child_frame_id = "lidar_odom";
////
////    curr_lidar_odom.pose.pose.position.x = pose_x_lidar;
////    curr_lidar_odom.pose.pose.position.y = pose_y_lidar;
////    curr_lidar_odom.pose.pose.position.z = pose_z_lidar;
////
////    curr_lidar_odom.pose.pose.orientation.x = pose_qx_lidar;
////    curr_lidar_odom.pose.pose.orientation.y = pose_qy_lidar;
////    curr_lidar_odom.pose.pose.orientation.z = pose_qz_lidar;
////    curr_lidar_odom.pose.pose.orientation.w = pose_qw_lidar;
////
////    lidar_odom_pub.publish(curr_lidar_odom);
//
////    // Using ROS TF
////    // get the position difference between two frames
////    double delta_pose_x = Transf(0, 3);
////    double delta_pose_y = Transf(1, 3);
////    double delta_pose_z = Transf(2, 3);
////
////    std::cout << "delta x in lidar_odom is " << delta_pose_x << std::endl;
////    std::cout << "delta y in lidar_odom is " << delta_pose_y << std::endl;
////    std::cout << "delta z in lidar_odom is " << delta_pose_z << std::endl;
////
////    // Update the current position with delta
////    pose_x_lidar = pose_x_lidar + delta_pose_x;
////    pose_y_lidar = pose_y_lidar + delta_pose_y;
////    pose_z_lidar = pose_z_lidar + delta_pose_z;
////
////    // Get the rotation matrix between two frames
////    tf::Matrix3x3 rotation_matrix;
////    rotation_matrix.setValue(static_cast<double>(Transf(0, 0)), static_cast<double>(Transf(0, 1)),
////                       static_cast<double>(Transf(0, 2)), static_cast<double>(Transf(1, 0)),
////                       static_cast<double>(Transf(1, 1)), static_cast<double>(Transf(1, 2)),
////                       static_cast<double>(Transf(2, 0)), static_cast<double>(Transf(2, 1)),
////                       static_cast<double>(Transf(2, 2)));
////
////    // Get euler angles (delta) from rotation matrix
////    double delta_pose_yaw;
////    double delta_pose_pitch;
////    double delta_pose_roll;
////    rotation_matrix.getRPY(delta_pose_roll, delta_pose_pitch, delta_pose_yaw);
////
////    // Update pose with euler angles (delta)
////    pose_yaw_lidar += delta_pose_yaw;
////    pose_pitch_lidar += delta_pose_pitch;
////    pose_roll_lidar += delta_pose_roll;
////
////    // Convert the updated euler angles to quaternion
//////    geometry_msgs::Quaternion curr_q_lidar;
//////    curr_q_lidar = tf::createQuaternionMsgFromRollPitchYaw(pose_roll_lidar, pose_pitch_lidar, pose_yaw_lidar);
////
////    tf::Quaternion curr_q_lidar;
////    curr_q_lidar.setRPY(pose_roll_lidar, pose_pitch_lidar, pose_yaw_lidar);
////    curr_q_lidar.normalize();
//////    std::cout << "quaternion of lidar_odom is " << curr_q_lidar << std::endl;
////
////    pose_qx_lidar = curr_q_lidar.x();
////    pose_qy_lidar = curr_q_lidar.y();
////    pose_qz_lidar = curr_q_lidar.z();
////    pose_qw_lidar = curr_q_lidar.w();
//
//    // Using transform matrix
//    pose_lidar = Transf*pose_lidar;
//
//    pose_x_lidar = pose_lidar(0, 3);
//    pose_y_lidar = pose_lidar(1, 3);
//    pose_z_lidar = pose_lidar(2, 3);
//
//    // Get the rotation matrix
//    Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Identity();
//    rotation_matrix = pose_lidar.block(0, 0, 3, 3);
//
//    // Convert the ratation matrix to quaternion
//    Eigen::Quaternionf curr_q_lidar = Eigen::Quaternionf(rotation_matrix);
//    Eigen::Vector4f curr_quat_vector = curr_q_lidar.coeffs(); // in order of x y z w
//
//    pose_qx_lidar = curr_quat_vector(0);
//    pose_qy_lidar = curr_quat_vector(1);
//    pose_qz_lidar = curr_quat_vector(2);
//    pose_qw_lidar = curr_quat_vector(3);
//
//    // publish the lidar odometry
//    nav_msgs::Odometry curr_lidar_odom;
//    curr_lidar_odom.header.stamp = curr_scan_msg.header.stamp;
//    curr_lidar_odom.header.frame_id = "imu_link";
//    curr_lidar_odom.child_frame_id = "lidar_odom";
//
//    curr_lidar_odom.pose.pose.position.x = pose_x_lidar;
//    curr_lidar_odom.pose.pose.position.y = pose_y_lidar;
//    curr_lidar_odom.pose.pose.position.z = pose_z_lidar;
//
//    curr_lidar_odom.pose.pose.orientation.x = pose_qx_lidar;
//    curr_lidar_odom.pose.pose.orientation.y = pose_qy_lidar;
//    curr_lidar_odom.pose.pose.orientation.z = pose_qz_lidar;
//    curr_lidar_odom.pose.pose.orientation.w = pose_qw_lidar;
//
//    lidar_odom_pub.publish(curr_lidar_odom);
//
////    static tf::TransformBroadcaster br_imu2lidar; // create a broadcaster for transform from imu to lidar_odom
////
////    static tf::Transform tf_imu2lidar; // Transform from imu_link to lidar_odom
////    tf_imu2lidar.setOrigin(tf::Vector3(pose_x_lidar, pose_y_lidar, pose_z_lidar)); // Translation between frames
////    tf::Quaternion q_imu2lidar = curr_q_lidar;
////    q_imu2lidar.normalize(); // normalize to enhance numeric stability
////    tf_imu2lidar.setRotation(q_imu2lidar);
////
//////    br_imu2lidar.sendTransform(tf::StampedTransform(tf_imu2lidar, ros::Time::now(), "imu_link", "lidar_odom")); // send the transform with time stamp
////    br_imu2lidar.sendTransform(tf::StampedTransform(tf_imu2lidar, curr_scan_msg.header.stamp, "imu_link", "lidar_odom"));
//////    std::cout << "Time Stamp of the lidar Message is " << curr_scan_msg.header.stamp << std::endl;
//
//    // Publish the Current Scan For Debug
//    pcl::copyPointCloud(*curr_scan_cloud_ptr, *pre_scan_cloud_ptr);// update the previous point cloud by current point cloud;
//    // Note:
//    // If direct copy is used, both of the two point cloud with be the same thing, which means,
//    // any changes on one will result in changes on the other
//
//    pose_lidar_pre = pose_lidar; // update the previous pose by current pose;
//
//    static sensor_msgs::PointCloud2 pre_scan_cloud_msg;
//    pcl::toROSMsg(*pre_scan_cloud_ptr, pre_scan_cloud_msg);
//    int num_point_prescan_msg = pre_scan_cloud_msg.width;
//    std::cout << "Size of the previous point cloud msg is " << num_point_prescan_msg << std::endl;
//    pre_scan_cloud_msg.header.frame_id = curr_scan_msg.header.frame_id;
//    scan_pub.publish(pre_scan_cloud_msg);
//
//
//
//    return;
//
//}

//static void PointCloud_Callback(const sensor_msgs::PointCloud2 curr_scan_msg)
//{
//    if (!gnss_odom_initialized)
//        // If the gnss_odom is not yet already initialized
//    {
//        std::cout << "PointCloud_Callback: gnss_odom is not yet initialized!" << std::endl;
//        return;
//    }
//    ROS_INFO("PointCloudCallback Started");
//    //Get current scan
//    pcl::fromROSMsg(curr_scan_msg, *curr_scan_cloud_ptr); //Convert point cloud msg to point cloud pointer for incoming laser scan
//
//    // Check if current scan is the first scan
//    if (pre_scan_cloud_ptr->width == 0)
//    {
//        std::cout << "Width of the previous scan is " << pre_scan_cloud_ptr->width << std::endl;
//        pcl::copyPointCloud(*curr_scan_cloud_ptr, *pre_scan_cloud_ptr);// update the previous point cloud by current point cloud;
//
//        // Get the pose of the vehicle under the frame of gnss_odom at this moment
//        // Using the pose from gnss directly
//
//        pose_x_lidar = pose_x_gnss;
//        pose_y_lidar = pose_y_gnss;
//        pose_z_lidar = pose_z_gnss;
//
//        pose_qx_lidar = pose_qx_gnss;
//        pose_qy_lidar = pose_qy_gnss;
//        pose_qz_lidar = pose_qz_gnss;
//        pose_qw_lidar = pose_qw_gnss;
//
//        q_lidar2imu = Eigen::Quaterniond(pose_qw_lidar, pose_qx_lidar, pose_qy_lidar, pose_qz_lidar); // Order: w, x, y, z
//        q_lidar2imu.normalize();
//        std::cout << "initial lidar quaternion is " << std::endl << q_lidar2imu.coeffs() << std::endl;
//        rotation_matrix_lidar2imu = q_lidar2imu.matrix();
//        std::cout << "initial lidar rotation matrix is " << std::endl << rotation_matrix_lidar2imu << std::endl;
//
//        // Initialize the pose of lidar
//        pose_lidar.rotate(q_lidar2imu);
//        //todo: check the sign
//        pose_lidar.pretranslate(Eigen::Vector3d(pose_x_lidar, pose_y_lidar, pose_z_lidar));
//        std::cout << "initial pose of the lidar " <<  std::endl << pose_lidar.matrix() << std::endl;
//
//        pose_lidar_pre = pose_lidar;
//
//        return;
//
//    }
//
//    //filter the incoming point cloud message
//    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
//    approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
//    approximate_voxel_filter.setInputCloud(curr_scan_cloud_ptr);
//    approximate_voxel_filter.filter(*curr_scan_cloud_ptr);
//    std::cout << "Filtered cloud contains " << curr_scan_cloud_ptr->size() << std::endl;
//
//    // expand the queue with the point cloud
//    point_cloud point_cloud_tmp;
//
//    point_cloud_tmp.point_cloud_ptr = curr_scan_cloud_ptr;
//    point_cloud_tmp.time = curr_scan_msg.header.stamp.toSec();
//    point_cloud_ptr_buffer.push_front(point_cloud_tmp);
//    if (point_cloud_ptr_buffer.size() > 5)
//    {
//        point_cloud_ptr_buffer.pop_back();
//    }
//    std::cout << "Number of Elements in POINT_CLOUD_PTR_QUEUE is " << point_cloud_ptr_buffer.size() << std::endl;
//
//    ROS_INFO("Scan Msg Converted");
//    std::cout << "Size of the previous Scan is " << pre_scan_cloud_ptr->width << std::endl;
//    std::cout << "Size of the current Scan is " << curr_scan_cloud_ptr->width << std::endl;
//
//    //Get transform between two frames
//
//    pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
//    ndt_omp->setInputSource(curr_scan_cloud_ptr);
//    ndt_omp->setInputTarget(pre_scan_cloud_ptr);
//    ndt_omp->setTransformationEpsilon (epsilon);
//    ndt_omp->setResolution (res);
//    ndt_omp->setStepSize (step);
//    ndt_omp->setMaximumIterations(100);
//
//    ndt_omp->setNumThreads(NumThreads);
//    ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
//    // get the transform between two poses as guess
////    Eigen::Matrix4f pose_diff = pose_lidar*(pose_lidar_pre.inverse());
//    ROS_INFO("Preparation for Scan Registration Finished");
//    ROS_INFO("Timer Started");
//    auto t1 = ros::WallTime::now();
//    ndt_omp->align(*aligned_scan_cloud_ptr);
//    auto t2 = ros::WallTime::now();
//    ROS_INFO("Timer Ended");
//    std::cout << "Duration : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;
//
//    std::cout << "Normal Distributions Transform has converged after " << ndt_omp->getFinalNumIteration() << " iters" << std::endl;
//    std::cout << "The score is " << ndt_omp->getFitnessScore() << "\n" << std::endl;
//
//    Eigen::Matrix4f Transf = Eigen::Matrix4f::Identity();
//    Transf = ndt_omp->getFinalTransformation();
//    std::cout << "Final Transformation Matrix of the Scan Registation is " << std::endl << Transf << std::endl;
//    cv::Mat t_pcl = (cv::Mat_<double>(3,1) << Transf(0, 3), Transf(1, 3), Transf(2, 3));
//    std::cout << "Mode of the translation is " << std::endl << cv::norm(t_pcl) << std::endl;
//    // todo: transform the point cloud to the current frame of lidar
////    Eigen::Matrix4d pose_diff;
////    pose_lidar_diff.matrix() = static_cast<Eigen::Matrix4d>(Transf);
//    pose_lidar_diff.matrix() = Transf.cast<double>();
//    // Using transform matrix
//    pose_lidar = pose_lidar_diff*pose_lidar;
//
//    pose_x_lidar = pose_lidar(0, 3);
//    pose_y_lidar = pose_lidar(1, 3);
//    pose_z_lidar = pose_lidar(2, 3);
//
//    // Get the rotation matrix
//    rotation_matrix_lidar2imu = pose_lidar.rotation();
//    // Convert the ratation matrix to quaternion
//    q_lidar2imu = Eigen::Quaterniond(rotation_matrix_lidar2imu);
//
//    // in order of x y z w
//    pose_qx_lidar = q_lidar2imu.coeffs()(0);
//    pose_qy_lidar = q_lidar2imu.coeffs()(1);
//    pose_qz_lidar = q_lidar2imu.coeffs()(2);
//    pose_qw_lidar = q_lidar2imu.coeffs()(3);
//
//    // publish the lidar odometry
//    nav_msgs::Odometry curr_lidar_odom;
//    curr_lidar_odom.header.stamp = curr_scan_msg.header.stamp;
//    curr_lidar_odom.header.frame_id = "imu_link";
//    curr_lidar_odom.child_frame_id = "lidar_odom";
//
//    curr_lidar_odom.pose.pose.position.x = pose_x_lidar;
//    curr_lidar_odom.pose.pose.position.y = pose_y_lidar;
//    curr_lidar_odom.pose.pose.position.z = pose_z_lidar;
//
//    curr_lidar_odom.pose.pose.orientation.x = pose_qx_lidar;
//    curr_lidar_odom.pose.pose.orientation.y = pose_qy_lidar;
//    curr_lidar_odom.pose.pose.orientation.z = pose_qz_lidar;
//    curr_lidar_odom.pose.pose.orientation.w = pose_qw_lidar;
//
//    lidar_odom_pub.publish(curr_lidar_odom);
//
////    static tf::TransformBroadcaster br_imu2lidar; // create a broadcaster for transform from imu to lidar_odom
////
////    static tf::Transform tf_imu2lidar; // Transform from imu_link to lidar_odom
////    tf_imu2lidar.setOrigin(tf::Vector3(pose_x_lidar, pose_y_lidar, pose_z_lidar)); // Translation between frames
////    tf::Quaternion q_imu2lidar = curr_q_lidar;
////    q_imu2lidar.normalize(); // normalize to enhance numeric stability
////    tf_imu2lidar.setRotation(q_imu2lidar);
////
//////    br_imu2lidar.sendTransform(tf::StampedTransform(tf_imu2lidar, ros::Time::now(), "imu_link", "lidar_odom")); // send the transform with time stamp
////    br_imu2lidar.sendTransform(tf::StampedTransform(tf_imu2lidar, curr_scan_msg.header.stamp, "imu_link", "lidar_odom"));
//////    std::cout << "Time Stamp of the lidar Message is " << curr_scan_msg.header.stamp << std::endl;
//
//    // Publish the Current Scan For Debug
//    pcl::copyPointCloud(*curr_scan_cloud_ptr, *pre_scan_cloud_ptr);// update the previous point cloud by current point cloud;
//    // Note:
//    // If direct copy is used, both of the two point cloud with be the same thing, which means,
//    // any changes on one will result in changes on the other
//
//    pose_lidar_pre = pose_lidar; // update the previous pose by current pose;
//
//    static sensor_msgs::PointCloud2 pre_scan_cloud_msg;
//    pcl::toROSMsg(*pre_scan_cloud_ptr, pre_scan_cloud_msg);
//    int num_point_prescan_msg = pre_scan_cloud_msg.width;
//    std::cout << "Size of the previous point cloud msg is " << num_point_prescan_msg << std::endl;
//    pre_scan_cloud_msg.header.frame_id = curr_scan_msg.header.frame_id;
//    scan_pub.publish(pre_scan_cloud_msg);
//    return;
//
//}

static void PointCloud_Callback_Class(const sensor_msgs::PointCloud2 curr_scan_msg)
{
    ROS_INFO("PointCloudCallback Started");
    //Get current scan
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(curr_scan_msg, *scan_cloud_ptr); //Convert point cloud msg to point cloud pointer for incoming laser scan
    std::cout << "original number of point cloud is " << scan_cloud_ptr->width << std::endl;

    if (!gnss_odom_initialized)
        // If the gnss_odom is not yet already initialized
    {
        std::cout << "PointCloud_Callback: gnss_odom is not yet initialized!" << std::endl;
        return;
    }

    // Check if current scan is the first scan
    if (gnss_odom_initialized && !map_ptr->curr_pc_frame_) // 当GNSS已经初始化而且curr pc frame为空（即还未收到gnss初始化之后的第一个lidar scan）
    {
        std::cout << "First scan after gnss initialization" << std::endl;

        // Get the pose of the vehicle under the frame of gnss_odom at this moment
        // Using the pose from gnss directly

//        pose_x_lidar = pose_x_gnss;
//        pose_y_lidar = pose_y_gnss;
//        pose_z_lidar = pose_z_gnss;
//
//        pose_qx_lidar = pose_qx_gnss;
//        pose_qy_lidar = pose_qy_gnss;
//        pose_qz_lidar = pose_qz_gnss;
//        pose_qw_lidar = pose_qw_gnss;
//
//        q_lidar2imu = Eigen::Quaterniond(pose_qw_lidar, pose_qx_lidar, pose_qy_lidar, pose_qz_lidar); // Order: w, x, y, z
//        q_lidar2imu.normalize();
//        std::cout << "initial lidar quaternion is " << std::endl << q_lidar2imu.coeffs() << std::endl;
//        rotation_matrix_lidar2imu = q_lidar2imu.matrix();
//        std::cout << "initial lidar rotation matrix is " << std::endl << rotation_matrix_lidar2imu << std::endl;
//
//        // Initialize the pose of lidar
//        pose_lidar.rotate(q_lidar2imu);
//        //todo: check the sign
//        pose_lidar.pretranslate(Eigen::Vector3d(pose_x_lidar, pose_y_lidar, pose_z_lidar));
//        std::cout << "initial pose of the lidar " <<  std::endl << pose_lidar.matrix() << std::endl;

        pc_frame::ptr pc_frame_ptr = pc_frame_ptr->create_frame();
        pc_frame_ptr->scan_ = scan_cloud_ptr;
        pc_frame_ptr->filter();
        pc_frame_ptr->T_cw_ = map_ptr->gnss_frames_.front()->T_cw_; // todo: front() or begin()
        std::cout << "id of the cloud point frame ptr is " << pc_frame_ptr->id_ << std::endl;
        map_ptr->add_keypcframe(pc_frame_ptr);
        map_ptr->update_lastpcframe();
        pc_frame_ptr->pub_odom(lidar_odom_pub);

        std::cout << "Finish processing of the first scan" << std::endl;

        return;
    }

    pc_frame::ptr pc_frame_ptr = pc_frame_ptr->create_frame();
    pc_frame_ptr->scan_ = scan_cloud_ptr;
    pc_frame_ptr->filter();

    std::cout << "id of the cloud point frame ptr is " << pc_frame_ptr->id_ << std::endl;
    map_ptr->add_keypcframe(pc_frame_ptr);

    //Get transform between two frames
    Eigen::Isometry3d pose_lidar_diff = Eigen::Isometry3d::Identity();
    map_ptr->register_keypcframes(pose_lidar_diff);
    // todo: transform the point cloud to the current frame of lidar

    map_ptr->update_lidar_pose(pose_lidar_diff);
    map_ptr->update_lastpcframe();

    Eigen::Vector3d vel_est = map_ptr->get_vel();
    pc_frame_ptr->pub_odom(lidar_odom_pub);
    return;

}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{

    ros::init(argc, argv, "ndt_node");
    ros::NodeHandle nh;

    // %Tag(SUBSCRIBER)%
    imu_sub = nh.subscribe("/kitti/oxts/imu", 100, IMU_Callback_Class);
    scan_sub = nh.subscribe("/kitti/velo/pointcloud", 10, PointCloud_Callback_Class);
    gnss_sub = nh.subscribe("/kitti/oxts/gps/fix", 100, GNSS_Callback_Class);
//    lcam_sub = nh.subscribe("/kitti/camera_color_left/image_raw", 10, LCAM_Callback);
//    lcam_sub_class = nh.subscribe("/kitti/camera_color_left/image_raw", 10, LCAM_Callback_Class);

    // %EndTag(SUBSCRIBER)%

    // %Tag(PUBLISHER)%
    chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
    lidar_odom_pub = nh.advertise<nav_msgs::Odometry>("/lidar_odom", 50);
    scan_pub = nh.advertise<sensor_msgs::PointCloud2>("/pre_scan", 10);
    gnss_odom_pub = nh.advertise<nav_msgs::Odometry>("/gnss_odom", 50);
    imu_odom_pub = nh.advertise<nav_msgs::Odometry>("/imu_odom", 50);
    lcam_odom_pub = nh.advertise<nav_msgs::Odometry>("/lcam_odom", 50);
    // %EndTag(PUBLISHER)%

    // Transform

    int count = 0;



    ros::Rate loop_rate(10);

    /**
    * ros::spin() will enter a loop, pumping callbacks.  With this version, all
    * callbacks will be called from within this thread (the main one).  ros::spin()
    * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
    */
    std_msgs::String msg;
    while(ros::ok()){


        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();
    // %EndTag(FILL_MESSAGE)%

    // %Tag(ROSCONSOLE)%
        ROS_INFO("%s", msg.data.c_str());
    // %EndTag(ROSCONSOLE)%


    // %Tag(PUBLISH)%
        chatter_pub.publish(msg);

    //    ros::spinOnce();

        ros::spin();

    //    loop_rate.sleep();

        count++;

    }

    return 0;
}
// %EndTag(FULLTEXT)%
