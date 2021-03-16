//
// Created by eagleflag on 2020/9/17.
//

#include "gnss_frame.h"

#include "map.h" // 因为gnss_frame和map要互相包含头文件，在gnss_frame.h里对map进行了forward declaration，这里要include map.h才能让编译器知道map这个类的具体情况

gnss_frame::gnss_frame()
{
    std::cout << "gnss_frame without id " << "is constructed" << std::endl;
}

gnss_frame::gnss_frame(
        unsigned long id,
        double time_stamp,
        Eigen::Isometry3d T_cw,
        gnss_data frame
):
        id_(id),
        time_stamp_(time_stamp),
        T_cw_(T_cw),
        frame_(frame)// Initialization of the constructor
{
    std::cout << "frame with id " << id_ << " is constructed" << std::endl;
}

gnss_frame::~gnss_frame()
{
    std::cout << "gnss_frame with id " << id_ << " is destructed" << std::endl;
}

gnss_frame::ptr gnss_frame::create_frame()
{
    static unsigned long gnss_frame_id = 0;
    return gnss_frame::ptr( new gnss_frame(gnss_frame_id++) ); // ptr here is the shared pointer defined with typedef
}



void gnss_frame::set_origin(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr, const std::shared_ptr<map>& map_ptr)
{

    geo_converter_.Reset(nav_sat_fix_ptr->latitude, nav_sat_fix_ptr->longitude, nav_sat_fix_ptr->altitude);// 把此时的gps位置设为原点)
    frame_.time = nav_sat_fix_ptr->header.stamp.toSec();
    frame_.latitude = nav_sat_fix_ptr->latitude;
    frame_.longitude = nav_sat_fix_ptr->longitude;
    frame_.altitude = nav_sat_fix_ptr->altitude;
    frame_.status = nav_sat_fix_ptr->status.status;
    frame_.service = nav_sat_fix_ptr->status.service;
    frame_.local_E = 0.0;
    frame_.local_N = 0.0;
    frame_.local_U = 0.0;

    double pose_qx = map_ptr->imu_frames_.front()->frame_.orientation_x;
    double pose_qy = map_ptr->imu_frames_.front()->frame_.orientation_y;
    double pose_qz = map_ptr->imu_frames_.front()->frame_.orientation_z;
    double pose_qw = map_ptr->imu_frames_.front()->frame_.orientation_w;

    Eigen::Quaterniond q = Eigen::Quaterniond(pose_qw, pose_qx, pose_qy, pose_qz); // Oder: w, x, y, z
    q.normalize();
    T_cw_.prerotate(q);
    T_cw_.pretranslate(Eigen::Vector3d(0.0, 0.0, 0.0));
}

void gnss_frame::set_frame(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr, const std::shared_ptr<map>& map_ptr)
{

    double pose_x, pose_y, pose_z;
    geo_converter_.Forward(nav_sat_fix_ptr->latitude, nav_sat_fix_ptr->longitude, nav_sat_fix_ptr->altitude, pose_x, pose_y, pose_z);

    frame_.time = nav_sat_fix_ptr->header.stamp.toSec();
    frame_.latitude = nav_sat_fix_ptr->latitude;
    frame_.longitude = nav_sat_fix_ptr->longitude;
    frame_.altitude = nav_sat_fix_ptr->altitude;
    frame_.status = nav_sat_fix_ptr->status.status;
    frame_.service = nav_sat_fix_ptr->status.service;

    frame_.local_E = pose_x;
    frame_.local_N = pose_y;
    frame_.local_U = pose_z;

    double pose_qx = map_ptr->imu_frames_.front()->frame_.orientation_x;
    double pose_qy = map_ptr->imu_frames_.front()->frame_.orientation_y;
    double pose_qz = map_ptr->imu_frames_.front()->frame_.orientation_z;
    double pose_qw = map_ptr->imu_frames_.front()->frame_.orientation_w;

    Eigen::Quaterniond q = Eigen::Quaterniond(pose_qw, pose_qx, pose_qy, pose_qz); // Oder: w, x, y, z
    q.normalize();
//    Eigen::Matrix3d rotation_update = q.toRotationMatrix();
//    Eigen::Vector3d translation_update = Eigen::Vector3d(pose_x, pose_y, pose_z);
//    std::cout << "Transform before update is "<< std::endl << T_cw_.matrix() << std::endl;
//    std::cout << "Update rotation is " << std::endl << rotation_update << std::endl;
//    std::cout << "Update translation is " << std::endl << translation_update << std::endl;

    T_cw_.prerotate(q);
    T_cw_.pretranslate(Eigen::Vector3d(pose_x, pose_y, pose_z));
//    std::cout << "Transform after update is "<< std::endl << T_cw_.matrix() << std::endl;
}


void gnss_frame::pub_odom(ros::Publisher odom_pub)
{
    std::cout << "Publishing the gnss odom "<< std::endl;
    // update the position relative to first frame
    double pose_x = T_cw_.matrix()(0, 3);
    double pose_y = T_cw_.matrix()(1, 3);
    double pose_z = T_cw_.matrix()(2, 3);

    // update the rotation matrix relative to first frame
//    Eigen::Matrix3d rotation_matrix = (T_cw_.matrix()).block<3, 3>(0, 0);
    Eigen::Matrix3d rotation_matrix = T_cw_.rotation();

    // Update the Quaternion
    Eigen::Quaterniond q = Eigen::Quaterniond(rotation_matrix);
    double pose_qx = q.coeffs()(0);
    double pose_qy = q.coeffs()(1);
    double pose_qz = q.coeffs()(2);
    double pose_qw = q.coeffs()(3);


    // publish the gnss odometry /////////////////////////////////////////////////////////////////////////////////////////
    nav_msgs::Odometry curr_odom;
    curr_odom.header.frame_id = "imu_link";
    curr_odom.child_frame_id = "gnss_odom";

    curr_odom.pose.pose.position.x = pose_x;
    curr_odom.pose.pose.position.y = pose_y;
    curr_odom.pose.pose.position.z = pose_z;

    curr_odom.pose.pose.orientation.x = pose_qx;
    curr_odom.pose.pose.orientation.y = pose_qy;
    curr_odom.pose.pose.orientation.z = pose_qz;
    curr_odom.pose.pose.orientation.w = pose_qw;

    odom_pub.publish(curr_odom);

    std::cout << "Finishing gnss odom Publishing "<< std::endl;
}

