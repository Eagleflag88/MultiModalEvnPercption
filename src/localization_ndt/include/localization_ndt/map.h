//
// Created by eagleflag on 2020/8/17.
//

#ifndef CATKIN_WS_NDT_MAP_H
#define CATKIN_WS_NDT_MAP_H


#include <Eigen/Geometry>
#include <Eigen/Core>
#include "frame.h"
#include "pc_frame.h"
#include "imu_frame.h"
#include "gnss_frame.h"
#include "mappoint.h"
#include "data_type.h"
#include <map>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/sfm/triangulation.hpp>
#include <opencv2/video/tracking.hpp>

#include "Geocentric/LocalCartesian.hpp"

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>


class map
{
public:
    typedef std::shared_ptr<map> ptr;
    std::map<unsigned long, frame::ptr> keyframes_ = {};
    frame::ptr pre_frame_ = nullptr;
    frame::ptr curr_frame_ = nullptr;
    std::map<unsigned long, pc_frame::ptr> keypcframes_ = {};
    pc_frame::ptr pre_pc_frame_ = nullptr;
    pc_frame::ptr curr_pc_frame_ = nullptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_pc_map_ = nullptr;

    unsigned long id_gnss_ini_ = 0;
    std::deque<imu_frame::ptr> imu_frames_ = {};
    std::deque<gnss_frame::ptr> gnss_frames_ = {};
    unsigned long id_imu_ini_ = 0;
    bool imu_initialized = false;

    std::vector<mappoint::ptr> mappoints_ = {};
    std::vector<frame::ptr> stereo_frames_ = {};
    std::vector<frame::ptr> stereo_pairs_ = {};
    bool first_binmatch_finished_ = false;

public:
    map();
    ~map();


    // For img_frame
    void add_keyimgframe(const frame::ptr newframe);
    void pose_estimation_2d2d(const frame::ptr pre_frame,
                              const frame::ptr curr_frame,
                              cv::Mat& R_opt,
                              cv::Mat& t_opt);
    void triangulation(const cv::Mat& R,
                       const cv::Mat& t);
    void ba_3d2d_optimization(const frame::ptr pre_frame,
                                   const frame::ptr curr_frame,
                                   const cv::Mat& R_ini,
                                   const cv::Mat& t_ini,
                                   cv::Mat& R_opt,
                                   cv::Mat& t_opt,
                                   std::vector<cv::Point3d>& points_3d_opt);
    void pose_estimation_pnp(const frame::ptr pre_frame,
                              const frame::ptr curr_frame,
                              cv::Mat& R,
                              cv::Mat& t);
    void feature_matches_2d2d();
    void lk_track(std::vector<unsigned char>& status);
    void update_lcampose(const cv::Mat& R,
                     const cv::Mat& t);
    void show_eulerangles(const cv::Mat& R);
    void update_lastimgframe();
    void update_lastimgpair();
    void crop_pairframe(const std::vector<unsigned char>& status);

    // For pc_frame
    void add_keypcframe(const pc_frame::ptr newpcframe);
    void register_keypcframes(Eigen::Isometry3d& pose_lidar_diff);
    void update_lidarpose(const Eigen::Isometry3d pose_lidar_diff);
    void update_lastpcframe();
    Eigen::Vector3d get_vel();
    void update_localpcmap();

    // For imu frame
    void update_imupose();
    void set_imu_origin(const gnss_frame::ptr gnss_frame_ptr); //

    // For gnss frame
    void add_keygnssframe(const gnss_frame::ptr new_gnssframe);

    // For Binocular Application
//    void add_mappoint(const frame::ptr left_frame,
//                           const frame::ptr right_frame);
    void process_binoimg();
    bool stereoframesready();
    void feature_matches_2d2d(frame::ptr& left_frame,
                              frame::ptr& right_frame);
    void find_3d2dmatches(frame::ptr& left_frame,
                          frame::ptr& right_frame);
    void find_3d3dmatches(frame::ptr& pre_left_frame,
                          frame::ptr& left_frame);
    int compute_descriptordist(const cv::Mat& a, const cv::Mat& b);
    void pose_estimation_pnp(const std::vector<mappoint::ptr>& mappoints_pre,
                                  const std::vector<cv::Point2f>& kps2f_curr,
                                  cv::Mat& R,
                                  cv::Mat& t);
    void pose_estimation_3d3d(
            const std::vector<cv::Point3d>& pts1,
            const std::vector<cv::Point3d>& pts2,
            cv::Mat& R,
            cv::Mat& t);
    void ba_3d3d(
            const std::vector< cv::Point3d >& pts1,
            const std::vector< cv::Point3d >& pts2,
            Eigen::Isometry3d& T);

    //todo: 释放内存
private:
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0.0, 325.1, 0.0, 521.0, 249.7, 0.0, 0.0, 1.0);

    const int NumThreads = 10;
    const double epsilon = 0.1;
    const int step = 1;
    const double res = 10.0;
    const int iter = 60;
    const Eigen::Vector3d accel_gravity = Eigen::Vector3d(0.0, 0.0, -9.8);
    const double time_diff_binocular = 0.1;
    const double b = 0.54; // todo: check
    const double fx = K.at<double>(0, 0); // todo: check
    const int descriptor_dist_thres = 30; // todo: check
    ros::Publisher bicam_odom_pub;

};

#endif //CATKIN_WS_NDT_MAP_H
