//
// Created by eagleflag on 2020/9/16.
//

#include "imu_frame.h"

imu_frame::imu_frame()
{
    std::cout << "imu_frame with id " << "is constructed" << std::endl;
}

imu_frame::imu_frame(
        unsigned long id,
        double time_stamp,
        Eigen::Isometry3d T_cw,
        Eigen::Matrix3d rotation_matrix,
        Eigen::Vector3d vel_global,
        Eigen::Vector3d accel_global,
        imu_data frame
):
        id_(id),
        time_stamp_(time_stamp),
        T_cw_(T_cw),
        rotation_matrix_(rotation_matrix),
        vel_global_(vel_global),
        accel_global_(accel_global),
        frame_(frame)// Initialization of the constructor
{
    std::cout << "imu_frame with id " << id_ << " is constructed" << std::endl;
}

imu_frame::~imu_frame()
{
    std::cout << "imu_frame with id " << id_ << " is destructed" << std::endl;
}

imu_frame::ptr imu_frame::create_frame()
{
    static unsigned long imu_frame_id = 0;
    return imu_frame::ptr( new imu_frame(imu_frame_id++) ); // ptr here is the shared pointer defined with typedef
}

void imu_frame::update_frame(const sensor_msgs::ImuConstPtr& imu_msg_ptr)
{
//    std::cout << "IMU Roll is " << pose_roll_imu << std::endl;
//    std::cout << "IMU PITCH is " << pose_pitch_imu << std::endl;
//    std::cout << "IMU YAW is " << pose_yaw_imu << std::endl;

    frame_.time = imu_msg_ptr->header.stamp.toSec();

    frame_.linear_acceleration_x = imu_msg_ptr->linear_acceleration.x; // m/s^2
    frame_.linear_acceleration_y = imu_msg_ptr->linear_acceleration.y;
    frame_.linear_acceleration_z = imu_msg_ptr->linear_acceleration.z;

    frame_.angular_velocity_x = imu_msg_ptr->angular_velocity.x; // rad/sec
    frame_.angular_velocity_y = imu_msg_ptr->angular_velocity.y;
    frame_.angular_velocity_z = imu_msg_ptr->angular_velocity.z;

    frame_.orientation_x = imu_msg_ptr->orientation.x;
    frame_.orientation_y = imu_msg_ptr->orientation.y;
    frame_.orientation_z = imu_msg_ptr->orientation.z;
    frame_.orientation_w = imu_msg_ptr->orientation.w;
}

void imu_frame::pub_odom(ros::Publisher odom_pub)
{
    std::cout << "Publishing the imu odom "<< std::endl;
    // update the position relative to first frame
    double pose_x = T_cw_.matrix()(0, 3);
    double pose_y = T_cw_.matrix()(1, 3);
    double pose_z = T_cw_.matrix()(2, 3);

    // update the rotation matrix relative to first frame
    Eigen::Matrix3d rotation_matrix = (T_cw_.matrix()).block<3, 3>(0, 0);

    // Update the Quaternion
    Eigen::Quaterniond q = Eigen::Quaterniond(rotation_matrix);
    double pose_qx = q.coeffs()(0);
    double pose_qy = q.coeffs()(1);
    double pose_qz = q.coeffs()(2);
    double pose_qw = q.coeffs()(3);


    // publish the imu odometry /////////////////////////////////////////////////////////////////////////////////////////
    nav_msgs::Odometry curr_odom;
    curr_odom.header.frame_id = "imu_link";
    curr_odom.child_frame_id = "imu_odom";

    curr_odom.pose.pose.position.x = pose_x;
    curr_odom.pose.pose.position.y = pose_y;
    curr_odom.pose.pose.position.z = pose_z;

    curr_odom.pose.pose.orientation.x = pose_qx;
    curr_odom.pose.pose.orientation.y = pose_qy;
    curr_odom.pose.pose.orientation.z = pose_qz;
    curr_odom.pose.pose.orientation.w = pose_qw;

    odom_pub.publish(curr_odom);

    std::cout << "Finishing imu odom Publishing "<< std::endl;
}
