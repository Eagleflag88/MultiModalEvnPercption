//
// Created by eagleflag on 2020/8/27.
//

#include "object_det.h"

object_det::object_det()
{
    std::cout << "object without id initialization" << "is constructed" << std::endl;
}

object_det::object_det(unsigned long id,
                       unsigned long frame_id,
                       Eigen::Vector3d pos,
                       Eigen::Vector3d pose, // Y, P, R
                       cv::Rect rect,
                       int CountDown,
                       cv::Mat bbox_img
                       ):
                       id_(id),
                       frame_id_(frame_id),
                       pos_(pos),
                       pose_(pose),
                       rect_(rect),
                       CountDown_(CountDown),
                       bbox_img_(bbox_img)

{
    std::cout << "object with id " << id_ << " is constructed" << std::endl;
}

object_det::~object_det()
{

}

object_det::ptr object_det::create_object()
{
    static unsigned long object_id = 0;
    return object_det::ptr( new object_det(object_id++) ); // ptr here is the shared pointer defined with typedef
}
