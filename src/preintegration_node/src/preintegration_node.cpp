//
// Created by eagleflag on 2021/3/22.
//

#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "imu_frame.h"

// Declaration of Publishers

static ros::Publisher chatter_pub;
static ros::Publisher imu_pub;

// Declaration of Subscribers
static ros::Subscriber imu_sub;

//todo: 查看所有容器的使用，复制还是引用
//todo: lock for data race

static void IMU_Callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr)
{
    imu_frame::ptr imu_frame_ptr = imu_frame_ptr->create_frame();
    imu_frame_ptr->update_frame(imu_msg_ptr);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "preintegration_node");
    ros::NodeHandle nh;
    // Register the Subscriber
    // todo:Add a parameter loading class

    chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

    int count = 0;
    std_msgs::String msg;
    while(ros::ok())
    {
        std::stringstream status_msg;
        status_msg << "preintegration_node working fine " << count;
        msg.data = status_msg.str();
        ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(msg);
        ros::spin();
        count++;
    }

    return 0;
}