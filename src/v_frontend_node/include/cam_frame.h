//
// Created by eagleflag on 2021/3/21.
//

#ifndef MULTIMODALEVNPERCPTION_CAM_FRAME_H
#define MULTIMODALEVNPERCPTION_CAM_FRAME_H

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

//#include "yolo_v2_class.hpp"
#include "object_det.h"
#include "NvInfer.h"
#include "NvInferRuntimeCommon.h"
#include "cuda_runtime.h"


class cam_frame
{

public:
    typedef std::shared_ptr<cam_frame> ptr;
    unsigned long id_;
    double time_stamp_;
    cv::Mat image_;

public:
    cam_frame();
    cam_frame(unsigned long id,
          double time_stamp = 0.0,
          cv::Mat image = cv::Mat()
); //constructor with default input, only id is not specified
    ~cam_frame();
    ptr create_frame();

};

#endif //MULTIMODALEVNPERCPTION_CAM_FRAME_H
