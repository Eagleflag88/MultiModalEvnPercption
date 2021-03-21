//
// Created by eagleflag on 2021/3/21.
//

#include "cam_frame.h"
#include <iostream>

cam_frame::cam_frame()
{
    std::cout << "camera frame without id " << "is constructed" << std::endl;
}

cam_frame::cam_frame(
        unsigned long id,
        double time_stamp,
        cv::Mat image
):
        id_(id),
        time_stamp_(time_stamp),
        image_(image)// Implementation of the constructor
{
    std::cout << "camera frame with id " << id_ << " is constructed" << std::endl;
}

cam_frame::~cam_frame()
{
    std::cout << "camera frame with id " << id_ << " is destructed" << std::endl;
}
cam_frame::ptr cam_frame::create_frame()
{
    static unsigned long cam_frame_id = 0;
    return cam_frame::ptr( new cam_frame(cam_frame_id++) ); // ptr here is the shared pointer defined with typedef
}