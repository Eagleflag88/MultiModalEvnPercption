//
// Created by eagleflag on 2021/3/21.
//

#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>


#include "tracker.h"

// Declaration of Publishers

static ros::Publisher image_pub;
static ros::Publisher chatter_pub;

// Declaration of Subscribers
static ros::Subscriber image_sub;

//todo: 查看所有容器的使用，复制还是引用
//todo: lock for data race

static void CAM_Callback(const sensor_msgs::ImageConstPtr& cam_img_msg_ptr)
{
    cv_bridge::CvImagePtr cam_cv_ptr = cv_bridge::toCvCopy(cam_img_msg_ptr);
    ROS_INFO("Get a image from camera");
    std::cout << "Number of column is " << cam_cv_ptr->image.cols << std::endl;
    cv::Mat curr_cam_img;
    curr_cam_img = cam_cv_ptr->image;
    std::cout << "the type of the read image is " << curr_cam_img.type() << std::endl;
//    cam_frame::ptr cam_frame_ptr = cam_frame_ptr->create_frame();
//    std::cout << "id of the left color cam_frame ptr is " << cam_frame_ptr->id_ << std::endl;
//    cam_frame_ptr->time_stamp_ = cam_img_msg_ptr->header.stamp.toSec();
//    cam_frame_ptr->image_ = curr_cam_img;

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "tracker_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    // Register the Subscriber
    // todo:Add a parameter loading class
    image_transport::Subscriber image_sub = it.subscribe("/kitti/camera_color_left/image_raw", 10, CAM_Callback);
    image_transport::Publisher image_pub = it.advertise("image_out", 1);
    chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

    tracker tracker;
    std::vector<object_det::ptr> new_objects;
    tracker.initialize(new_objects);
    int count = 0;
    std_msgs::String msg;
    while(ros::ok())
    {
        std::stringstream status_msg;
        status_msg << "tracker_node working fine " << count;
        msg.data = status_msg.str();
        ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(msg);
        tracker.update(new_objects);

        ros::spin();
        count++;
    }

    return 0;
}