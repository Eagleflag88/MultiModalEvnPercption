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
static ros::Subscriber rcam_sub;

static int count_estimation_lcam = 1;

bool gnss_odom_initialized = false;

static int count_lcam_callback = 0;
static int count_rcam_callback = 0;
static bool first_match_finished = false;

std::shared_ptr<map> map_ptr(new map());
tracker tracker_2d;

GeographicLib::LocalCartesian gnss_frame::geo_converter_ = GeographicLib::LocalCartesian(0.0, 0.0, 0.0);

//todo: 查看所有容器的使用，复制还是引用
//todo: lock for data race

static void IMU_Callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr)
{
    imu_frame::ptr imu_frame_ptr = imu_frame_ptr->create_frame();
    imu_frame_ptr->update_frame(imu_msg_ptr);
    map_ptr->imu_frames_.push_front(imu_frame_ptr); // 从前端插入
    map_ptr->update_imupose(); //在插入最新的imu_frame之后调用
    imu_frame_ptr->pub_odom(imu_odom_pub);
    std::cout << "Number of Elements in IMU_DATA_QUEUE is " << map_ptr->imu_frames_.size() << std::endl;
}

static void GNSS_Callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr)
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
        std::cout << "initial pose of left camera " <<  std::endl << gnss_frame_ptr->T_cw_.matrix() << std::endl;

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

}

static void LCAM_Callback(const sensor_msgs::ImageConstPtr& lcam_img_msg_ptr)
{
    if (!gnss_odom_initialized)
    {
        std::cout << "gnss_odom is not yet initialized" << std::endl;
        return;
    }

    count_lcam_callback += 1;
    cv_bridge::CvImagePtr lcam_cv_ptr = cv_bridge::toCvCopy(lcam_img_msg_ptr);
    ROS_INFO("Get a image from left camera");
    std::cout << "Number of column is " << lcam_cv_ptr->image.cols << std::endl;
    cv::Mat curr_lcam_img;
    curr_lcam_img = lcam_cv_ptr->image;
    std::cout << "the type of the read image is " << curr_lcam_img.type() << std::endl;
    frame::ptr frame_ptr = frame_ptr->create_frame();
    std::cout << "id of the left color frame ptr is " << frame_ptr->id_ << std::endl;
    frame_ptr->time_stamp_ = lcam_img_msg_ptr->header.stamp.toSec();
    frame_ptr->source_ = 0; // Left image
    frame_ptr->image_ = curr_lcam_img;
    //    map_ptr->add_keyimgframe(frame_ptr);

    // For Binocular Processing
    map_ptr->stereo_frames_.push_back(frame_ptr);
    std::cout << "One image from left camera is added" << std::endl;
    map_ptr->process_binoimg();




//    // For Tracking ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    std::vector<object_det::ptr> new_objects;
//    map_ptr->curr_frame_->detect_yolo_dark(new_objects);
//    map_ptr->curr_frame_->detect_yolo_trt(new_objects);
//    if (count_lcam_callback == 1)
//    {
//        tracker_2d.initialize(new_objects);
//    }
//    else
//    {
//        tracker_2d.update(new_objects);
//    }
//    tracker_2d.draw_tracks(map_ptr->curr_frame_);

//    if (!gnss_odom_initialized)
//        // if the gnss_odom is not yet initialized,
//        // no lcam_odom should be published (what will happen in the section below);
//    {
//        std::cout << "no lcam_odom should be published" << std::endl;
//        return;
//    }
//    // Check if current image is the first image after gnss initialization
//    if (gnss_odom_initialized && map_ptr->id_gnss_ini_ == 0) // 当GNSS已经初始化而且还未进行第一次特征匹配
//    {
//        std::cout << "First image after gnss initialization" << std::endl;
//        frame_ptr->T_cw_ = map_ptr->gnss_frames_.front()->T_cw_; // 设定整个lcam_odom的初始位姿，来自于gnss
//        map_ptr->id_gnss_ini_ = frame_ptr->id_;
//        return;
//    }
//
//    if(!first_match_finished && map_ptr->id_gnss_ini_ != 0) // 当还未进行第一次特征匹配,但是map已经初始化位姿
//    {
//        std::cout<<"It is the first matching!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1"<<std::endl;
//
//        // 2D-2D Feature Matching
//        map_ptr->feature_matches_2d2d();
//        // keypoints, kps2f, descriptors updated for pre and curr frame
//        std::cout << "size of keypoint in pre and curr frame are " << map_ptr->pre_frame_->keypoints_.size() << " " << map_ptr->curr_frame_->keypoints_.size() << std::endl;
//        std::cout << "size of kps2f in pre and curr frame are " << map_ptr->pre_frame_->kps2f_.size() << " " << map_ptr->curr_frame_->kps2f_.size() << std::endl;
//        std::cout << "size of descriptors in pre and curr frame are " << map_ptr->pre_frame_->descriptors_.rows << " " << map_ptr->curr_frame_->descriptors_.rows << std::endl;
//
//        //-- 估计两张图像间运动
//        cv::Mat R, t;
//        map_ptr->pose_estimation_2d2d(map_ptr->pre_frame_, map_ptr->curr_frame_, R, t);
//
//        //-- 三角化
//        map_ptr->triangulation(R, t);
//        // points_landmark updated for pre and (transformed into) curr frame
//
//        // 3d-2d Feature Matching through pnp: 3d point in frame of first camera, 2d feature in second camera ////////////////////////////////////
//        // Points_landmark should be represented in the frame of the first camera
//        cv::Mat R_pnp, t_pnp;
//        map_ptr->pose_estimation_pnp(map_ptr->pre_frame_, map_ptr->curr_frame_, R_pnp, t_pnp);
//        // nothing updated
//
//        map_ptr->show_eulerangles(R_pnp);
//
//        map_ptr->update_lcampose(R_pnp, t_pnp);
//        std::cout << "size of keypoint in pre and curr frame are " << map_ptr->pre_frame_->keypoints_.size() << " " << map_ptr->curr_frame_->keypoints_.size() << std::endl;
//        std::cout << "size of kps2f in pre and curr frame are " << map_ptr->pre_frame_->kps2f_.size() << " " << map_ptr->curr_frame_->kps2f_.size() << std::endl;
//        std::cout << "size of descriptors in pre and curr frame are " << map_ptr->pre_frame_->descriptors_.rows << " " << map_ptr->curr_frame_->descriptors_.rows << std::endl;
//
//        map_ptr->update_lastimgpair();
//
//        std::cout << "size of keypoint in pre and curr frame are " << map_ptr->pre_frame_->keypoints_.size() << " " << map_ptr->curr_frame_->keypoints_.size() << std::endl;
//        std::cout << "size of kps2f in pre and curr frame are " << map_ptr->pre_frame_->kps2f_.size() << " " << map_ptr->curr_frame_->kps2f_.size() << std::endl;
//        std::cout << "size of descriptors in pre and curr frame are " << map_ptr->pre_frame_->descriptors_.rows << " " << map_ptr->curr_frame_->descriptors_.rows << std::endl;
//
//        map_ptr->curr_frame_->pub_odom(lcam_odom_pub);
//
//        first_match_finished = true;
//        return;
//    }
//
//    // Track the keypoint with optical flow ////////////////////////////////////////////////////////////////////////////////////////////
//    std::cout << "Here comes a new image for 3d-2d matching!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
//    // Calculate the optical flow and get the tracked keypoint
//    std::vector<unsigned char> status;
//    map_ptr->lk_track(status);
//    // keypoints, kps2f, descriptors and landmark points updated for curr frame
//
//    map_ptr->crop_pairframe(status);
//
//    // Draw the tracked keypoints and store the image of the current image
//    map_ptr->curr_frame_->draw_and_store_tracking();
//
//    // 3d-2d Feature Matching through pnp: 3d point in frame of first camera, 2d feature in second camera ////////////////////////////////////
//    // Points_landmark should be represented in the frame of the first camera
//    cv::Mat R_pnp, t_pnp;
//    map_ptr->pose_estimation_pnp(map_ptr->pre_frame_, map_ptr->curr_frame_, R_pnp, t_pnp);
//    // nothing updated
//
//    map_ptr->show_eulerangles(R_pnp);
//
//    // Update the pose ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    map_ptr->update_lcampose(R_pnp, t_pnp);
//
//    //-- 三角化
//    map_ptr->triangulation(R_pnp, t_pnp);
//    // points_landmark updated for pre and (transformed into) curr frame
//
//    map_ptr->update_lastimgframe();
//
//    map_ptr->curr_frame_->pub_odom(lcam_odom_pub);
}

static void RCAM_Callback(const sensor_msgs::ImageConstPtr& rcam_img_msg_ptr)
{
    if (!gnss_odom_initialized)
    {
        std::cout << "gnss_odom is not yet initialized" << std::endl;
        return;
    }
    count_rcam_callback += 1;
    cv_bridge::CvImagePtr rcam_cv_ptr = cv_bridge::toCvCopy(rcam_img_msg_ptr);
    ROS_INFO("Get a image from right camera");
    std::cout << "Number of column is " << rcam_cv_ptr->image.cols << std::endl;
    cv::Mat curr_rcam_img;
    curr_rcam_img = rcam_cv_ptr->image;
    std::cout << "the type of the read image is " << curr_rcam_img.type() << std::endl;
    frame::ptr frame_ptr = frame_ptr->create_frame();
    std::cout << "id of the right color frame ptr is " << frame_ptr->id_ << std::endl;
    frame_ptr->time_stamp_ = rcam_img_msg_ptr->header.stamp.toSec();
    frame_ptr->source_ = 1; // Right image
    frame_ptr->image_ = curr_rcam_img;
    map_ptr->stereo_frames_.push_back(frame_ptr);
    std::cout << "One image from right camera is added" << std::endl;
    map_ptr->process_binoimg();

}

static void PointCloud_Callback(const sensor_msgs::PointCloud2 curr_scan_msg)
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

        pc_frame::ptr pc_frame_ptr = pc_frame_ptr->create_frame();
        pc_frame_ptr->scan_ = scan_cloud_ptr;
        pc_frame_ptr->filter();
        pc_frame_ptr->T_cw_ = map_ptr->gnss_frames_.front()->T_cw_;
        std::cout << "id of the cloud point frame ptr is " << pc_frame_ptr->id_ << std::endl;

        // 修改gnss的pose之后不会影响lidar的pose
//        std::cout << "current lidar pose before is "<< std::endl << pc_frame_ptr->T_cw_.matrix() << std::endl;
//        std::cout << "current gnss pose before is "<< std::endl << map_ptr->gnss_frames_.front()->T_cw_.matrix() << std::endl;
//        pc_frame_ptr->T_cw_.pretranslate(Eigen::Vector3d(1.0, 1.0, 1.0));
//        std::cout << "current lidar pose after is "<< std::endl << pc_frame_ptr->T_cw_.matrix() << std::endl;
//        std::cout << "current gnss pose after is "<< std::endl << map_ptr->gnss_frames_.front()->T_cw_.matrix() << std::endl;

        map_ptr->add_keypcframe(pc_frame_ptr);
        map_ptr->update_lastpcframe();
        map_ptr->update_localpcmap();
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

    map_ptr->update_lidarpose(pose_lidar_diff);
    map_ptr->update_lastpcframe();
    map_ptr->update_localpcmap();

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
    imu_sub = nh.subscribe("/kitti/oxts/imu", 100, IMU_Callback);
//    scan_sub = nh.subscribe("/kitti/velo/pointcloud", 10, PointCloud_Callback);
    gnss_sub = nh.subscribe("/kitti/oxts/gps/fix", 100, GNSS_Callback);
    lcam_sub = nh.subscribe("/kitti/camera_color_left/image_raw", 10, LCAM_Callback);
    rcam_sub = nh.subscribe("/kitti/camera_color_right/image_raw", 10, RCAM_Callback);


    // %EndTag(SUBSCRIBER)%

    // %Tag(PUBLISHER)%
    chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
    lidar_odom_pub = nh.advertise<nav_msgs::Odometry>("/lidar_odom", 50);
    scan_pub = nh.advertise<sensor_msgs::PointCloud2>("/pre_scan", 10);
    gnss_odom_pub = nh.advertise<nav_msgs::Odometry>("/gnss_odom", 50);
    imu_odom_pub = nh.advertise<nav_msgs::Odometry>("/imu_odom", 50);
    lcam_odom_pub = nh.advertise<nav_msgs::Odometry>("/lcam_odom", 50);
    // %EndTag(PUBLISHER)%


    int count = 0;

//    ros::Rate loop_rate(10);

    std_msgs::String msg;
    while(ros::ok())
    {
        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(msg);
        ros::spin();
        count++;
    }

    return 0;
}
// %EndTag(FULLTEXT)%
