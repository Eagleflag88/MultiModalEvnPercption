//
// Created by eagleflag on 2020/8/18.
//

#include "mappoint.h"
#include <iostream>

mappoint::mappoint()
{

}

mappoint::~mappoint()
{

}

mappoint::mappoint(unsigned long id,
                   Eigen::Vector3d pos,
                   unsigned int num_observed,
                   cv::Point2f kp2f,
                   cv::Mat descriptor,
                   cv::KeyPoint keypoint
                   ):
                   id_(id),
                   pos_(pos),
                   num_observed_(num_observed),
                   kp2f_(kp2f),
                   descriptor_(descriptor),
                   keypoint_(keypoint)
{
    std::cout << "mappoint with id " << id_ << " is constructed" << std::endl;
}

mappoint::ptr mappoint::create()
{
    static unsigned long mappoint_id = 0;
    return mappoint::ptr( new mappoint(mappoint_id++) );
}


