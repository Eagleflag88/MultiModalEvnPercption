//
// Created by eagleflag on 2020/8/18.
//

#ifndef CATKIN_WS_NDT_MAPPOINT_H
#define CATKIN_WS_NDT_MAPPOINT_H

#include <opencv2/core/core.hpp>
#include <Eigen/Geometry>
#include <Eigen/Core>

class mappoint
{

public:
    typedef std::shared_ptr<mappoint> ptr;
    unsigned long id_;
    Eigen::Vector3d pos_;
    unsigned int num_observed_;
    cv::Point2f kp2f_;
    cv::Mat descriptor_;
    cv::KeyPoint keypoint_;

public:
    mappoint();
    mappoint(unsigned long id,
             Eigen::Vector3d pos = Eigen::Vector3d(1, 1, 1),
             unsigned int num_observed = 0,
             cv::Point2f kp2f = cv::Point2f(),
             cv::Mat descriptor = cv::Mat(),
             cv::KeyPoint keypoint = cv::KeyPoint());
    ~mappoint();

    mappoint::ptr create();
};

#endif //CATKIN_WS_NDT_MAPPOINT_H
