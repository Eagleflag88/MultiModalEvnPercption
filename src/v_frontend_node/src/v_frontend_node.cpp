//
// Created by eagleflag on 2021/3/21.
//

#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "cam_frame.h"


// Declaration of Publishers

static ros::Publisher image_pub;
static ros::Publisher chatter_pub;

// Declaration of Subscribers
static ros::Subscriber image_sub;

//todo: 查看所有容器的使用，复制还是引用
//todo: lock for data race

static void IMG_Callback(const sensor_msgs::ImageConstPtr& cam_img_msg_ptr)
{
    cv_bridge::CvImagePtr cam_cv_ptr = cv_bridge::toCvCopy(cam_img_msg_ptr);
    ROS_INFO("Get a image from cam_node");
    std::cout << "Number of column is " << cam_cv_ptr->image.cols << std::endl;
    cv::Mat curr_cam_img;
    curr_cam_img = cam_cv_ptr->image;

    cam_frame::ptr frame_ptr = frame_ptr->create_frame();
    std::cout << "id of the camera frame ptr is " << frame_ptr->id_ << std::endl;
    frame_ptr->time_stamp_ = cam_img_msg_ptr->header.stamp.toSec();
    frame_ptr->image_ = curr_cam_img;
    std::vector<object_det::ptr> new_objects;
    frame_ptr->detect_yolo_dark(new_objects);
//    frame_ptr->detect_yolo_trt(new_objects);
    std::cout << "the type of the read image is " << curr_cam_img.type() << std::endl;

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "v_frontend_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    // Register the Subscriber
    // todo:Add a parameter loading class
    image_transport::Subscriber image_sub = it.subscribe("/kitti/camera_color_left/image_raw", 10, IMG_Callback);
    image_transport::Publisher image_pub = it.advertise("image_out", 1);
    chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

    int count = 0;
    std_msgs::String msg;
    while(ros::ok())
    {
        std::stringstream status_msg;
        status_msg << "v_frontend_node working fine " << count;
        msg.data = status_msg.str();
        ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(msg);
        ros::spin();
        count++;
    }

    return 0;
}