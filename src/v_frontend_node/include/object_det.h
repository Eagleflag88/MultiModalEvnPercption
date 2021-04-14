//
// Created by eagleflag on 2020/8/27.
//

#ifndef CATKIN_WS_NDT_OBJECT_H
#define CATKIN_WS_NDT_OBJECT_H

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>



class object_det {

public:
    typedef std::shared_ptr<object_det> ptr;
    unsigned long id_;
    unsigned long frame_id_;
    Eigen::Vector3d pos_;
    Eigen::Vector3d pose_; // Y, P, R
    cv::Rect rect_; // x, y, h, w
    int CountDown_;
    cv::Mat bbox_img_;


public:
    object_det();
    object_det(unsigned long id,
               unsigned long frame_id = 0,
               Eigen::Vector3d pos = Eigen::Vector3d(0.0, 0.0, 0.0),
               Eigen::Vector3d pose = Eigen::Vector3d(0.0, 0.0, 1.0),
               cv::Rect rect = cv::Rect(),
               int CountDown_ = 2,
               cv::Mat bbox_img_ = cv::Mat());
    ~object_det();
    ptr create_object();

};


#endif //CATKIN_WS_NDT_OBJECT_H
