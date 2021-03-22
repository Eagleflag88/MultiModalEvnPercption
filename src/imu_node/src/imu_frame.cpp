//
// Created by eagleflag on 2021/3/22.
//

#include "imu_frame.h"

imu_frame::imu_frame()
{
    std::cout << "imu_frame with id " << "is constructed" << std::endl;
}

imu_frame::imu_frame(
        unsigned long id,
        double time_stamp,
        imu_data frame
):
        id_(id),
        time_stamp_(time_stamp),
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

